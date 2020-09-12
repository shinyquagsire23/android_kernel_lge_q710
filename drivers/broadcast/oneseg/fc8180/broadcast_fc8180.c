#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>         /* wake_lock, unlock */
#include <linux/version.h>          /* check linux version */

#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <linux/err.h>
#include <linux/of_gpio.h>

#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>

#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/async.h>
#include <linux/types.h>
#include <linux/string.h>

typedef long intptr_t;

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8180.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "bbm.h"
#include "fc8180_drv_api.h"
#include "fc8180_isr.h"
#include "fci_ringbuffer.h"

#define LGE_FC8180_DRV_VER "1.00.02"

struct ISDBT_INIT_INFO_T *hInit;

#define FC8180_NAME        "broadcast1"

#define RING_BUFFER_SIZE	(188 * 320)

static int broadcast_spi_remove(struct spi_device *spi);
static int broadcast_spi_probe(struct spi_device *spi);
static int isdbt_pinctrl_init(void);
static int isdbt_set_gpio_config(void);
static int isdbt_free_gpio_config(void);

static int broadcast_check_chip_id(void);

static struct task_struct *isdbt_kthread;
struct fci_ringbuffer		RingBuffer;
static wait_queue_head_t isdbt_isr_wait;
u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static u8 isdbt_isr_start=0;
u32 totalTS=0;
u32 totalErrTS=0;

enum ISDBT_MODE{
	ISDBT_POWERON       = 0,
	ISDBT_POWEROFF	    = 1,
	ISDBT_DATAREAD		= 2
};
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;

u8 static_ringbuffer[RING_BUFFER_SIZE];
static DEFINE_MUTEX(ringbuffer_lock);

static struct of_device_id isdbt_fc8180_table[] = {
{
	.compatible = "fci,fc8180-spi",}, //Compatible node must match dts
	{ },
};

static struct spi_driver broadcast_spi_driver = {
	.probe = broadcast_spi_probe,
	.remove	= __broadcast_dev_exit_p(broadcast_spi_remove),
	.driver = {
		.name = "fc8180-spi",
		.of_match_table = isdbt_fc8180_table,
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

struct broadcast_fc8180_ctrl_data
{
    boolean                           pwr_state;
    struct spi_device*           spi_dev;
    struct wakeup_source      wake_lock;
    struct clk                      *clk;
    struct platform_device*   pdev;

    struct pinctrl                   *isdbt_pinctrl;
    struct pinctrl_state          *gpio_state_active;
    struct pinctrl_state          *gpio_state_sleep;

    int32                             isdbt_en;

    int32                             isdbt_ant_sw;
    int32                             isdbt_ant_active_mode;

    int32                             isdbt_lna_ctrl; //gain control
    int32                             isdbt_lna_en; //enable

    int32                             isdbt_en_lna_en_same_gpio;
    int32                             isdbt_ldo_en;

    int32                             isdbt_use_xtal;
    int32                             isdbt_xtal_freq;
    int32                             isdbt_interface_freq;

    int32                             ctrl_isdbt_ldo;
    struct regulator              *isdbt_ldo;
    int32                             ctrl_lna_ldo;
    struct regulator              *lna_ldo;
    int32                             ctrl_ant_sw_ldo;
    struct regulator              *ant_sw_ldo;
};

static struct broadcast_fc8180_ctrl_data  IsdbCtrlInfo;

struct spi_device* FCI_GET_SPI_DRIVER(void)
{
	return IsdbCtrlInfo.spi_dev;
}

static Device_drv device_fc8180 = {
    &broadcast_fc8180_drv_if_power_on,
    &broadcast_fc8180_drv_if_power_off,
    &broadcast_fc8180_drv_if_open,
    &broadcast_fc8180_drv_if_close,
    &broadcast_fc8180_drv_if_set_channel,
    &broadcast_fc8180_drv_if_resync,
    &broadcast_fc8180_drv_if_detect_sync,
    &broadcast_fc8180_drv_if_get_sig_info,
    &broadcast_fc8180_drv_if_get_ch_info,
    &broadcast_fc8180_drv_if_get_dmb_data,
    &broadcast_fc8180_drv_if_reset_ch,
    &broadcast_fc8180_drv_if_user_stop,
    &broadcast_fc8180_drv_if_select_antenna,
    &broadcast_fc8180_drv_if_isr,
    &broadcast_fc8180_drv_if_read_control,
    &broadcast_fc8180_drv_if_get_mode,
};

static int isdbt_pinctrl_init(void)
{
    IsdbCtrlInfo.isdbt_pinctrl = devm_pinctrl_get(&(IsdbCtrlInfo.pdev->dev));

    if(IS_ERR_OR_NULL(IsdbCtrlInfo.isdbt_pinctrl)) {
        pr_err("[dtv] Getting pinctrl handle failed\n");
        return -EINVAL;
    } else {
        printk("[dtv] Getting pinctrl handle succed\n");
    }

    IsdbCtrlInfo.gpio_state_active
     = pinctrl_lookup_state(IsdbCtrlInfo.isdbt_pinctrl, "isdbt_pin_active");

     if(IS_ERR_OR_NULL(IsdbCtrlInfo.gpio_state_active)) {
         pr_err("[dtv] Failed to get the active state pinctrl handle\n");
         return -EINVAL;
    } else {
        printk("[dtv] Getting active state pinctrl handle succed\n");
    }

    IsdbCtrlInfo.gpio_state_sleep
     = pinctrl_lookup_state(IsdbCtrlInfo.isdbt_pinctrl, "isdbt_pin_sleep");

     if(IS_ERR_OR_NULL(IsdbCtrlInfo.gpio_state_sleep)) {
         pr_err("[dtv] Failed to get the sleep state pinctrl handle\n");
         return -EINVAL;
    } else {
        printk("[dtv] Getting sleep state pinctrl handle succed\n");
    }

    return 0;
}

static int isdbt_set_gpio_config(void)
{
    if(pinctrl_select_state(IsdbCtrlInfo.isdbt_pinctrl, IsdbCtrlInfo.gpio_state_active)) {
        pr_err("[dtv] error on pinctrl_select_state to active\n");
        return -EINVAL;
    }
    else {
        printk("[dtv] success to set pinctrl_select_state to active\n");
    }

    return 0;
}

static int isdbt_free_gpio_config(void)
{
    if(pinctrl_select_state(IsdbCtrlInfo.isdbt_pinctrl, IsdbCtrlInfo.gpio_state_sleep)) {
        pr_err("[dtv] error on pinctrl_select_state to sleep\n");
        return -EINVAL;
    }
    else {
        printk("[dtv] success to set pinctrl_select_state to sleep\n");
    }

    return 0;
}

int fc8180_power_on(void)
{
    int i = 0;
    int rc = OK;

    rc = isdbt_set_gpio_config();
    printk("[dtv] isdbt_set_gpio_config=%d\n", rc);
    mdelay(5);

    if(IsdbCtrlInfo.pwr_state != TRUE)
    {
        __pm_stay_awake(&IsdbCtrlInfo.wake_lock);

        while (driver_mode == ISDBT_DATAREAD) {
            ms_wait(100);
            printk("[dtv] ISDBT_DATARREAD mode i=(%d)\n", i);
            if(i++ > 5)
                break;
        }

        rc = isdbt_ldo_power_on();
        printk("[dtv] isdbt_ldo_power_on=%d\n", rc);

        if(IsdbCtrlInfo.isdbt_en > 0) {
            gpio_set_value(IsdbCtrlInfo.isdbt_en, 0);
            mdelay(5);
            gpio_set_value(IsdbCtrlInfo.isdbt_en, 1);
            mdelay(5);
        }

        //[TDMBDEV-2766] (issue) DMB_EN control that FC8080 LDO_EN & 1.8V regulator
        //So add 100ms delay for power sequence before clk enable
        if(IsdbCtrlInfo.isdbt_en_lna_en_same_gpio == TRUE) {
            mdelay(100);
        }

        if(IsdbCtrlInfo.isdbt_use_xtal == FALSE) {
            if(IsdbCtrlInfo.clk != NULL) {
                printk("[dtv] fc8180_power_on() clk enable\n");
                rc = clk_prepare_enable(IsdbCtrlInfo.clk);
                if(rc) {
                    gpio_set_value(IsdbCtrlInfo.isdbt_en, 0);
                    msleep(30);
                    dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not enable clock\n");
                    return rc;
                }
            }
        }
        // For T1 time (>= 1msec)
        msleep(30);

        rc = isdbt_lna_power_on();
        printk("[dtv] isdbt_lna_power_on=%d\n", rc);
        rc = isdbt_ant_sw_power_on();
        printk("[dtv] isdbt_ant_sw_power_on=%d\n", rc);     

        // For T4 time (>=20 msec)
        msleep(30);

        driver_mode = ISDBT_POWERON;
    }
    else
    {
	printk("[dtv] already on!! \n");
    }

    IsdbCtrlInfo.pwr_state = TRUE;
    return rc;
}

int fc8180_is_power_on()
{
    return (int)IsdbCtrlInfo.pwr_state;
}

unsigned int fc8180_get_xtal_freq(void)
{
    return (unsigned int)IsdbCtrlInfo.isdbt_xtal_freq;
}

int fc8180_stop(void)
{
    if (driver_mode == ISDBT_DATAREAD) {
            driver_mode = ISDBT_POWEROFF;
            ms_wait(200);
    }
    driver_mode = ISDBT_POWEROFF;

    return OK;
}

int fc8180_power_off(void)
{
    int rc = OK;
    driver_mode = ISDBT_POWEROFF;

    if (IsdbCtrlInfo.pwr_state == 0) {
	printk("[dtv] power is immediately off\n");

	return OK;
    }
    else {
        printk("[dtv] power_off\n");

        isdbt_lna_power_off();
        isdbt_ant_sw_power_off();

        if(IsdbCtrlInfo.isdbt_use_xtal == FALSE) {
            if(IsdbCtrlInfo.clk != NULL) {
                clk_disable_unprepare(IsdbCtrlInfo.clk);
            }
        }

        if(IsdbCtrlInfo.isdbt_en > 0) {
            gpio_set_value(IsdbCtrlInfo.isdbt_en, 0);
        }

        msleep(10);

        isdbt_ldo_power_off();
    }

    __pm_relax(&IsdbCtrlInfo.wake_lock);

    rc = isdbt_free_gpio_config();
    printk("[dtv] isdbt_free_gpio_config=%d\n", rc);

    IsdbCtrlInfo.pwr_state = 0;
    return rc;
}

int isdbt_ldo_power_on(void)
{
    int rc = OK;

    if(IsdbCtrlInfo.ctrl_isdbt_ldo == TRUE) {
        rc = regulator_enable(IsdbCtrlInfo.isdbt_ldo);
        if(rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "unable to enable isdbt_ldo\n");
            return rc;
        } else {
            printk("[dtv] isdbt_ldo enable\n");
        }
    }

    if(IsdbCtrlInfo.isdbt_ldo_en > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_ldo_en, 1);
        printk("[dtv] set isdbt_use_ldo_en to 1\n");
        mdelay(10);
    }
    return rc;
}

void isdbt_ldo_power_off(void)
{
    if(IsdbCtrlInfo.isdbt_ldo_en > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_ldo_en, 0);
        printk("[dtv] set isdbt_use_ldo_en to 0\n");
        mdelay(10);
    }

    if(IsdbCtrlInfo.ctrl_isdbt_ldo == TRUE) {
        regulator_disable(IsdbCtrlInfo.isdbt_ldo);
        printk("[dtv] isdbt_ldo disable\n");
    }
}

int isdbt_lna_power_on(void)
{
    int rc = OK;
    if(IsdbCtrlInfo.ctrl_lna_ldo == TRUE) {
        rc = regulator_enable(IsdbCtrlInfo.lna_ldo);
        if(rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "unable to enable lna_ldo\n");
            return rc;
        } else {
            printk("[dtv] lna_ldo enable\n");
        }
    }

    if(IsdbCtrlInfo.isdbt_lna_ctrl > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_lna_ctrl, 0);
    }

    if(IsdbCtrlInfo.isdbt_lna_en > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_lna_en, 1);
    }

    return rc;
}

int isdbt_ant_sw_power_on(void)
{
    int rc = OK;
    if (IsdbCtrlInfo.ctrl_ant_sw_ldo == TRUE) {
        rc = regulator_enable(IsdbCtrlInfo.ant_sw_ldo);
        if(rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "unable to enable ant_sw_ldo\n");
            return rc;
        } else {
            printk("[dtv] ant_sw_ldo enable\n");
        }
    }

    if(IsdbCtrlInfo.isdbt_ant_sw > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_ant_sw, IsdbCtrlInfo.isdbt_ant_active_mode);
        printk("[dtv] isdbt_ant_sw_gpio_value : %d\n", IsdbCtrlInfo.isdbt_ant_active_mode);
    }

    return rc;
}

void isdbt_lna_power_off(void)
{
    if(IsdbCtrlInfo.isdbt_lna_ctrl > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_lna_ctrl, 0);
    }

    if(IsdbCtrlInfo.isdbt_lna_en > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_lna_en, 0);
    }

    if(IsdbCtrlInfo.ctrl_lna_ldo == TRUE) {
        regulator_disable(IsdbCtrlInfo.lna_ldo);
        printk("[dtv] lna_ldo disable\n");
    }
}

void isdbt_ant_sw_power_off(void)
{
    if(IsdbCtrlInfo.isdbt_ant_sw > 0) {
        gpio_set_value(IsdbCtrlInfo.isdbt_ant_sw, !(IsdbCtrlInfo.isdbt_ant_active_mode));
        printk("[dtv] isdbt_ant_sw_gpio_value : %d\n", !(IsdbCtrlInfo.isdbt_ant_active_mode));
    }

    if(IsdbCtrlInfo.ctrl_ant_sw_ldo == TRUE) {
        regulator_disable(IsdbCtrlInfo.ant_sw_ldo);
        printk("[dtv] ant_sw_ldo disable\n");
    }
}

static void broadcast_isdbt_config_gpios(void)
{
    IsdbCtrlInfo.isdbt_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,en-gpio", 0);
    if(IsdbCtrlInfo.isdbt_en < 0) {
        printk("[dtv] not setting or failed get isdbt_en = %d\n", IsdbCtrlInfo.isdbt_en);
    } else {
        printk("[dtv] isdbt_en = %d\n",IsdbCtrlInfo.isdbt_en);
    }

    IsdbCtrlInfo.isdbt_ant_sw = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,ant-gpio", 0);
    if(IsdbCtrlInfo.isdbt_ant_sw < 0) {
        printk("[dtv] not setting or failed get isdbt_ant_sw = %d\n", IsdbCtrlInfo.isdbt_ant_sw);        
    } else {
        printk("[dtv] isdbt_ant_sw = %d\n", IsdbCtrlInfo.isdbt_ant_sw);
        of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ant-active-mode", &IsdbCtrlInfo.isdbt_ant_active_mode);
    	 printk("[dtv] isdbt_ant_active_mode : %d\n", IsdbCtrlInfo.isdbt_ant_active_mode);
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "isdbt-en-lna-en-same-gpio", &IsdbCtrlInfo.isdbt_en_lna_en_same_gpio);
    printk("[dtv]  isdbt_en_lna_en_same_gpio : %d\n", IsdbCtrlInfo.isdbt_en_lna_en_same_gpio);

    IsdbCtrlInfo.isdbt_ldo_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,ldo-en-gpio", 0);
    if(IsdbCtrlInfo.isdbt_ldo_en < 0) {
        printk("[dtv] not setting or failed get isdbt_ldo_en = %d\n", IsdbCtrlInfo.isdbt_ldo_en);
    } else {
        printk("[dtv] isdbt_ldo_en = %d\n", IsdbCtrlInfo.isdbt_ldo_en);
    }

    IsdbCtrlInfo.isdbt_lna_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,lna-en-gpio",0); /* EN */
    if(IsdbCtrlInfo.isdbt_lna_en < 0) {
        printk("[dtv] not setting or failed get isdbt_lna_en = %d\n", IsdbCtrlInfo.isdbt_lna_en);
    } else {
        printk("[dtv] isdbt_lna_en = %d\n",  IsdbCtrlInfo.isdbt_lna_en);
    }

    IsdbCtrlInfo.isdbt_lna_ctrl = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,lna-ctrl-gpio",0); /* GC */
    if(IsdbCtrlInfo.isdbt_lna_ctrl < 0) {
        printk("[dtv] not setting or failed get isdbt_lna_ctrl = %d\n", IsdbCtrlInfo.isdbt_lna_ctrl);
    } else {
        printk("[dtv] isdbt_lna_ctrl = %d\n", IsdbCtrlInfo.isdbt_lna_ctrl);
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "use-xtal", &IsdbCtrlInfo.isdbt_use_xtal);
    printk("[dtv] use_xtal : %d\n", IsdbCtrlInfo.isdbt_use_xtal);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "xtal-freq", &IsdbCtrlInfo.isdbt_xtal_freq);
    printk("[dtv] isdbt_xtal_freq : %d\n", IsdbCtrlInfo.isdbt_xtal_freq);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "interface-freq", &IsdbCtrlInfo.isdbt_interface_freq);
    IsdbCtrlInfo.spi_dev->max_speed_hz     = (IsdbCtrlInfo.isdbt_interface_freq*1000);
    printk("[dtv] isdbt_interface_freq : %d\n", IsdbCtrlInfo.isdbt_interface_freq);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ctrl-isdbt-ldo", &IsdbCtrlInfo.ctrl_isdbt_ldo);
    printk("[dtv] ctrl_isdbt_ldo : %d\n", IsdbCtrlInfo.ctrl_isdbt_ldo);

    if(IsdbCtrlInfo.ctrl_isdbt_ldo == TRUE) {
        IsdbCtrlInfo.isdbt_ldo = devm_regulator_get(&IsdbCtrlInfo.spi_dev->dev, "isdbt-ldo");
        if(IS_ERR(IsdbCtrlInfo.isdbt_ldo)) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "regulator isdbt_ldo failed\n");
        }
        else {
            printk("[dtv] isdbt_ldo regulator OK\n");
        }
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ctrl-lna-ldo", &IsdbCtrlInfo.ctrl_lna_ldo);
    printk("[dtv] ctrl_lna_ldo : %d\n", IsdbCtrlInfo.ctrl_lna_ldo);

    if(IsdbCtrlInfo.ctrl_lna_ldo == TRUE) {
        IsdbCtrlInfo.lna_ldo= devm_regulator_get(&IsdbCtrlInfo.spi_dev->dev, "lna-ldo");
        if(IS_ERR(IsdbCtrlInfo.lna_ldo)) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "regulator lna_ldo failed\n");
        }
        else {
            printk("[dtv] lna_ldo regulator OK\n");
        }
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ctrl-ant-sw-ldo", &IsdbCtrlInfo.ctrl_ant_sw_ldo);
    printk("[dtv] ctrl_ant_sw_ldo : %d\n", IsdbCtrlInfo.ctrl_ant_sw_ldo);

    if(IsdbCtrlInfo.ctrl_ant_sw_ldo == TRUE) {
        IsdbCtrlInfo.ant_sw_ldo= devm_regulator_get(&IsdbCtrlInfo.spi_dev->dev, "ant-sw-ldo");
        if(IS_ERR(IsdbCtrlInfo.ant_sw_ldo)) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "regulator lna_ldo failed\n");
        }
        else {
            printk("[dtv] ant_sw_ldo regulator OK\n");
        }
    }
}

unsigned int fc8180_get_ts(void *buf, unsigned int size)
{
    s32 avail;
    ssize_t len, total_len = 0;

    if (fci_ringbuffer_empty(&RingBuffer))
        return 0;

    mutex_lock(&ringbuffer_lock);

    avail = fci_ringbuffer_avail(&RingBuffer);

    if (size >= avail)
        len = avail;
    else
        len = size - (size % 188);

    total_len = fci_ringbuffer_read_user(&RingBuffer, buf, len);

    mutex_unlock(&ringbuffer_lock);

    return total_len;
}

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
    if ((driver_mode == ISDBT_POWEROFF) || (isdbt_isr_start)) {
        printk("fc8180 isdbt_irq : abnormal Interrupt\
            occurred fc8180 driver_mode : %d state.cnt : %d\n", driver_mode, irq_error_cnt);
        irq_error_cnt++;
        isdbt_isr_start = 0;
    } else {
        printk("fc8180 isdbt_irq : %d\n", isdbt_isr_sig);

        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }

    return IRQ_HANDLED;
}

int data_callback(u32 hDevice, u8 *data, int len)
{
	int i;
    unsigned long ts_error_count = 0;
	totalTS += (len / 188);

	for (i = 0; i < len; i += 188) {
		if ((data[i+1] & 0x80) || data[i] != 0x47)
			ts_error_count++;
	}

    if (ts_error_count > 0) {
		totalErrTS += ts_error_count;
		printk("[dtv] data_callback totalErrTS\
			: %d, len : %d\n", totalErrTS, len);
    }

	mutex_lock(&ringbuffer_lock);

	if (fci_ringbuffer_free(&RingBuffer) < len)
		FCI_RINGBUFFER_SKIP(&RingBuffer, len);

	fci_ringbuffer_write(&RingBuffer, data, len);
	
	wake_up_interruptible(&(RingBuffer.queue));

	mutex_unlock(&ringbuffer_lock);

	return 0;
}

static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

#ifdef MTK_FTRACE_TEST
	unsigned long previous_time = 0;
	unsigned int isdbt_ftrace_mode = 0;
#endif

	set_user_nice(current, -20);

	print_log(hInit, "[dtv] isdbt_kthread enter\n");

	bbm_com_ts_callback_register((intptr_t)hInit, data_callback);

	init_waitqueue_head(&isdbt_isr_wait);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait
			, isdbt_isr_sig || kthread_should_stop());

		if (driver_mode == ISDBT_POWERON) {
                    driver_mode = ISDBT_DATAREAD;

                    //print_log(hInit, "isdbt_kthread isr\n");

                    bbm_com_isr(NULL);
                    if (driver_mode == ISDBT_DATAREAD)
                        driver_mode = ISDBT_POWERON;
		}

		if (isdbt_isr_sig > 0) {
#ifdef MTK_FTRACE_TEST
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0) {
				if (isdbt_ftrace_mode == 0) {
					if (isdbt_isr_sig > 1) {
						tracing_off();
						isdbt_ftrace_mode = 1;
					} else if (isdbt_isr_sig) {
						if ((previous_time) &&
							((unsigned long)jiffies
							- previous_time)
							< 300) {
							tracing_off();
							isdbt_ftrace_mode = 1;
						}
						previous_time
						= (unsigned long)jiffies;
					}
				}
				isdbt_isr_sig = 0;
			}
#else
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0)
				isdbt_isr_sig = 0;
#endif
		}

		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	printk("[dtv] isdbt_kthread exit\n");

	return 0;
}

void fci_irq_disable(void)
{
    isdbt_isr_sig = 0;

    printk("[dtv] fci_irq_disable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void fci_irq_enable(void)
{
    isdbt_isr_sig = 0;

    printk("[dtv] fci_irq_enable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void broadcast_fci_ringbuffer_flush(void)
{
	fci_ringbuffer_flush(&RingBuffer);
}

static int broadcast_spi_remove(struct spi_device *spi)
{
    printk("[dtv] broadcast_spi_remove\n");

    free_irq(spi->irq, NULL);

    if (isdbt_kthread) {
        kthread_stop(isdbt_kthread);
        isdbt_kthread = NULL;
    }
    bbm_com_hostif_deselect(NULL);

    wakeup_source_trash(&IsdbCtrlInfo.wake_lock);

    return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
    int rc = 0;

    printk("[dtv] broadcast_spi_probe start\n");

    if(spi == NULL) {
        printk("[dtv] spi is NULL, so spi can not be set\n");
        return -1;
    }

    spi->mode 			= SPI_MODE_0;
    IsdbCtrlInfo.spi_dev = spi;
    spi->bits_per_word	= 8;
    IsdbCtrlInfo.pdev = to_platform_device(&spi->dev);

    /* Config GPIOs : IRQ PIN Output and 0 set at initial */
    broadcast_isdbt_config_gpios();

    rc = spi_setup(spi);
    if (rc) {
        printk("[dtv] Spi setup error(%d)\n", rc);
        return rc;
    }

    printk("[dtv] IsdbCtrlInfo.isdbt_use_xtal %d, %d\n", IsdbCtrlInfo.isdbt_use_xtal, IsdbCtrlInfo.isdbt_xtal_freq);

    if(IsdbCtrlInfo.isdbt_use_xtal == FALSE) {
        IsdbCtrlInfo.clk = clk_get(&IsdbCtrlInfo.spi_dev->dev, "isdbt_xo");
  	
        if(IS_ERR(IsdbCtrlInfo.clk)){
            rc = PTR_ERR(IsdbCtrlInfo.clk);
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(IsdbCtrlInfo.clk);
        if (rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(IsdbCtrlInfo.clk);
    }

    wakeup_source_init(&IsdbCtrlInfo.wake_lock, dev_name(&spi->dev));

    fci_ringbuffer_init(&RingBuffer, &static_ringbuffer[0], RING_BUFFER_SIZE);

    if (!isdbt_kthread) {
        printk("[dtv] kthread run\n");
        isdbt_kthread = kthread_run(isdbt_thread, NULL, "isdbt_thread");
    }

    isdbt_pinctrl_init();

//Use to request_threaded_irq with kerenl version 3.18
//    rc = request_irq(spi->irq, isdbt_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, NULL);

//Use to request_threaded_irq with kerenl version 4.9
    rc = request_threaded_irq(spi->irq, NULL, isdbt_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &IsdbCtrlInfo);

    if (rc){
        printk("[dtv] dmb request irq fail : %d\n", rc);
    }

    rc = broadcast_dmb_drv_start(&device_fc8180);
    if (rc) {
        printk("[dtv] Failed to load Device (%d)\n", rc);
        rc = ERROR;
    }

    if (rc < 0)
        goto free_irq;

    fci_irq_disable(); /* Must disabled */

    if(broadcast_check_chip_id() != OK) {
        printk("[dtv] Chip ID read fail!!\n");
        rc = ERROR;
    }

    printk("[dtv] broadcast_spi_probe End.\n");
    return 0;

free_irq:
    broadcast_spi_remove(IsdbCtrlInfo.spi_dev);

    return rc;
}

static int broadcast_check_chip_id(void)
{
    int rc = ERROR;

    rc = fc8180_power_on();

    rc |= bbm_com_hostif_select(NULL, BBM_SPI);
    print_log(0, "[dtv] bbm_com_hostif_select rc : %d \n", rc);

    rc |= bbm_com_i2c_init(NULL, FCI_HPI_TYPE);
    print_log(0, "[dtv] bbm_com_i2c_init rc : %d \n", rc);

#ifdef BBM_SPI_IF
    rc |= bbm_com_byte_write(NULL, BBM_DM_DATA, 0x00);
    print_log(0, "[dtv] bbm_com_byte_write rc : %d \n", rc);
#endif

    rc |= bbm_com_probe(NULL);
    print_log(0, "[dtv] bbm_com_probe rc : %d \n", rc);

    rc |= fc8180_power_off();

    if(rc != OK)
        rc = ERROR;

    return rc;
}

static void async_fc8080_drv_init(void *data, async_cookie_t cookie)
{
    int rc;

    rc = spi_register_driver(&broadcast_spi_driver);
    if (rc < 0)
        printk("[dtv] SPI driver register failed(%d)\n", rc);

    return;
}

static int __broadcast_dev_init broadcast_dmb_fc8180_drv_init(void)
{
    int ret = 0;

    if(broadcast_dmb_drv_check_module_init() != OK) {
        ret = ERROR;
        return ret;
    }

    async_schedule(async_fc8080_drv_init, NULL);

    printk("Linux Version : %d\n", LINUX_VERSION_CODE);
    printk("LGE_FC8180_DRV_VER : %s\n", LGE_FC8180_DRV_VER);

    return ret;
}

static void __exit broadcast_dmb_fc8180_drv_exit(void)
{
    spi_unregister_driver(&broadcast_spi_driver);
    printk("[dtv] %s\n", __func__);
}

module_init(broadcast_dmb_fc8180_drv_init);
module_exit(broadcast_dmb_fc8180_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("FCI");
