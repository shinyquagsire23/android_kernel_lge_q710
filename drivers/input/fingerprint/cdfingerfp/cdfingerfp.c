#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/pm_wakeup.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>


typedef struct key_report {
	int key;
	int value;
}key_report_t;

struct cdfinger_key_map {
	unsigned int type;
	unsigned int code;
};

#define CDFINGER_IOCTL_MAGIC_NO				0xFB
#define CDFINGER_INIT						_IOW(CDFINGER_IOCTL_MAGIC_NO, 0, uint8_t)
#define CDFINGER_GETIMAGE					_IOW(CDFINGER_IOCTL_MAGIC_NO, 1, uint8_t)
#define CDFINGER_INITERRUPT_MODE			_IOW(CDFINGER_IOCTL_MAGIC_NO, 2, uint8_t)
#define CDFINGER_INITERRUPT_KEYMODE			_IOW(CDFINGER_IOCTL_MAGIC_NO, 3, uint8_t)
#define CDFINGER_INITERRUPT_FINGERUPMODE	_IOW(CDFINGER_IOCTL_MAGIC_NO, 4, uint8_t)
#define CDFINGER_RELEASE_WAKELOCK			_IO(CDFINGER_IOCTL_MAGIC_NO, 5)
#define CDFINGER_CHECK_INTERRUPT			_IO(CDFINGER_IOCTL_MAGIC_NO, 6)
#define CDFINGER_SET_SPI_SPEED				_IOW(CDFINGER_IOCTL_MAGIC_NO, 7, uint32_t)
#define CDFINGER_REPORT_KEY_LEGACY			_IOW(CDFINGER_IOCTL_MAGIC_NO, 10, uint8_t)
#define CDFINGER_POWERDOWN					_IO(CDFINGER_IOCTL_MAGIC_NO, 11)
#define	CDFINGER_GETID						_IO(CDFINGER_IOCTL_MAGIC_NO,12)

#define CDFINGER_INIT_GPIO					_IO(CDFINGER_IOCTL_MAGIC_NO,20)
#define CDFINGER_INIT_IRQ					_IO(CDFINGER_IOCTL_MAGIC_NO,21)
#define CDFINGER_POWER_ON					_IO(CDFINGER_IOCTL_MAGIC_NO,22)
#define CDFINGER_RESET						_IO(CDFINGER_IOCTL_MAGIC_NO,23)
#define CDFINGER_POWER_OFF					_IO(CDFINGER_IOCTL_MAGIC_NO,24)
#define CDFINGER_RELEASE_DEVICE				_IO(CDFINGER_IOCTL_MAGIC_NO,25)

#define CDFINGER_DISABLE_IRQ				_IO(CDFINGER_IOCTL_MAGIC_NO, 13)
#define CDFINGER_HW_RESET					_IOW(CDFINGER_IOCTL_MAGIC_NO, 14, uint8_t)
#define CDFINGER_GET_STATUS					_IO(CDFINGER_IOCTL_MAGIC_NO, 15)
#define	CDFINGER_REPORT_KEY					_IOW(CDFINGER_IOCTL_MAGIC_NO,19,key_report_t)
#define CDFINGER_NEW_KEYMODE				_IOW(CDFINGER_IOCTL_MAGIC_NO, 37, uint8_t)
#define CDFINGER_WAKE_LOCK					_IOW(CDFINGER_IOCTL_MAGIC_NO,26,uint8_t)
#define CDFINGER_ENABLE_IRQ					_IOW(CDFINGER_IOCTL_MAGIC_NO, 27, uint8_t)


#define DEVICE_NAME "fpsdev0"
#define INPUT_DEVICE_NAME "cdfinger_inputdev"

#ifdef CONFIG_LGE_ONE_BINARY_SKU
#define SUPPORT_ONE_BINARY
#endif

//#define SUPPORT_FP_ID_DUAL

#ifdef SUPPORT_ONE_BINARY
extern int lge_get_fingerprint_hwinfo(void);
#endif // SUPPORT_ONE_BINARY

#ifndef FP_ID_VALUE
#define FP_ID_VALUE (1) // FP_ID is default high
#endif

//#define POWER_GPIO
#define POWER_REGULATOR

static int isInKeyMode = 0; // key mode
static int screen_status = 1; // screen on
static u8 cdfinger_debug = 0x01;
static int isInit = 0;

#define LOG_TAG "[cdfinger] "

#define CDFINGER_DBG(fmt, args...) \
	do { \
		if (cdfinger_debug & 0x01) \
			printk(KERN_INFO LOG_TAG "%s: " fmt, __func__, ##args ); \
	} while(0)

#define CDFINGER_ERR(fmt, args...) \
	do { \
		printk(KERN_ERR LOG_TAG "%s: " fmt, __func__, ##args ); \
	} while(0)

struct cdfingerfp_data {
	struct platform_device *cdfinger_dev;
	struct miscdevice *miscdev;
#ifdef SUPPORT_FP_ID_DUAL
	int fp_id;
	u32 id_num;
	u8 chip_id;
	struct pinctrl *fps_pinctrl;
	struct pinctrl_state *fps_id_high;
#endif // SUPPORT_FP_ID_DUAL
	u32 irq_num;
	u32 reset_num;
#ifdef POWER_GPIO
	u32 pwr_num;
#endif
#ifdef POWER_REGULATOR
	struct regulator *vdd;
#endif
	struct fasync_struct *async_queue;
	struct input_dev* cdfinger_input;
	struct notifier_block notifier;
	struct mutex buf_lock;
	int irq_enable_status;
}*g_cdfingerfp_data;

static struct cdfinger_key_map maps[] = {
	{ EV_KEY, KEY_UP },
	{ EV_KEY, KEY_DOWN },
	{ EV_KEY, KEY_LEFT },
	{ EV_KEY, KEY_RIGHT },
	{ EV_KEY, KEY_ENTER },
};

static int cdfinger_init_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	CDFINGER_DBG("enter\n");
#ifdef POWER_GPIO
	if (gpio_is_valid(cdfinger->pwr_num)) {
		err = gpio_request(cdfinger->pwr_num, "cdfinger-pwr");
		if (err) {
			gpio_free(cdfinger->pwr_num);
			err = gpio_request(cdfinger->pwr_num, "cdfinger-pwr");
			if (err) {
				CDFINGER_ERR("Could not request pwr gpio.\n");
				return err;
			}
		}
		gpio_direction_output(cdfinger->pwr_num, 1);
	}
	else {
		CDFINGER_ERR("not valid pwr gpio\n");
		return -EIO;
	}
#endif

	if (gpio_is_valid(cdfinger->reset_num)) {
		err = gpio_request(cdfinger->reset_num, "cdfinger-reset");
		if (err) {
			gpio_free(cdfinger->reset_num);
			err = gpio_request(cdfinger->reset_num, "cdfinger-reset");
			if (err) {
				CDFINGER_ERR("Could not request reset gpio.\n");
#ifdef POWER_GPIO
				gpio_free(cdfinger->pwr_num);
#endif
				return err;
			}
		}
		gpio_direction_output(cdfinger->reset_num, 1);
	}
	else {
		CDFINGER_ERR("not valid reset gpio\n");
#ifdef POWER_GPIO
		gpio_free(cdfinger->pwr_num);
#endif
		return -EIO;
	}

	if (gpio_is_valid(cdfinger->irq_num)) {
		err = gpio_request(cdfinger->irq_num, "cdfinger-irq");
		if (err) {
			gpio_free(cdfinger->irq_num);
			err = gpio_request(cdfinger->irq_num, "cdfinger-irq");
			if (err) {
				CDFINGER_ERR("Could not request irq gpio.\n");
				gpio_free(cdfinger->reset_num);
#ifdef POWER_GPIO
				gpio_free(cdfinger->pwr_num);
#endif
				return err;
			}
		}
		gpio_direction_input(cdfinger->irq_num);
	}
	else {
		CDFINGER_ERR("not valid irq gpio\n");
		gpio_free(cdfinger->reset_num);
#ifdef POWER_GPIO
		gpio_free(cdfinger->pwr_num);
#endif
		return -EIO;
	}

	return err;
}

static int cdfinger_free_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	CDFINGER_DBG("enter\n");

	if (gpio_is_valid(cdfinger->irq_num)) {
		if (isInit == 1) {
			free_irq(gpio_to_irq(cdfinger->irq_num), (void*)cdfinger);
			isInit = 0;
		}
		gpio_free(cdfinger->irq_num);
	}

	if (gpio_is_valid(cdfinger->reset_num)) {
		gpio_free(cdfinger->reset_num);
	}
#ifdef POWER_GPIO
	if (gpio_is_valid(cdfinger->pwr_num)) {
		gpio_free(cdfinger->pwr_num);
	}
#endif

	return err;
}

static void cdfinger_reset(struct cdfingerfp_data *pdata, int ms)
{
	gpio_set_value(pdata->reset_num, 1);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 0);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 1);
	mdelay(ms);
}

static int cdfinger_parse_dts(struct device *dev, struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	CDFINGER_DBG("enter\n");
#ifdef POWER_GPIO
	cdfinger->pwr_num = of_get_named_gpio(dev->of_node, "cdfinger,pwr_gpio", 0);
#endif
	cdfinger->reset_num = of_get_named_gpio(dev->of_node, "cdfinger,reset_gpio", 0);
	cdfinger->irq_num = of_get_named_gpio(dev->of_node, "cdfinger,irq_gpio", 0);

#ifdef SUPPORT_FP_ID_DUAL
	cdfinger->id_num = of_get_named_gpio(dev->of_node, "cdfinger,id_gpio", 0);
	cdfinger->fps_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cdfinger->fps_pinctrl))
	{
		CDFINGER_ERR("pinctrl get failed!\n");
		err = -1;
	}
#endif // SUPPORT_FP_ID_DUAL
	return err;
}

static int cdfinger_power_on(struct cdfingerfp_data *pdata)
{
	int ret = 0;

	CDFINGER_DBG("enter\n");

#ifdef POWER_GPIO
//	gpio_direction_output(pdata->pwr_num, 1);
	gpio_set_value(pdata->reset_num, 1);
	mdelay(10);

#endif
#ifdef POWER_REGULATOR
	regulator_set_voltage(pdata->vdd, 2800000, 2800000);
	ret = regulator_enable(pdata->vdd);
	if (ret)
	{
		CDFINGER_ERR("enable regulato fail\n");
		return ret;
	}
#endif
	msleep(10);

	CDFINGER_DBG("exit\n");

	return ret;
}

static int cdfinger_power_off(struct cdfingerfp_data *pdata)
{
	CDFINGER_DBG("enter\n");

#ifdef POWER_GPIO
//	gpio_direction_output(pdata->pwr_num, 0);
	gpio_set_value(pdata->reset_num, 0);
	mdelay(10);
#endif
#ifdef POWER_REGULATOR
	regulator_disable(pdata->vdd);
#endif
	mdelay(1);

	CDFINGER_DBG("exit\n");

	return 0;
}

static int cdfinger_open(struct inode *inode, struct file *file)
{
	CDFINGER_DBG("enter\n");

	file->private_data = g_cdfingerfp_data;

	CDFINGER_DBG("exit\n");

	return 0;
}

static int cdfinger_async_fasync(int fd, struct file *file,int mode)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	return fasync_helper(fd,file,mode,&cdfingerfp->async_queue);
}

static int cdfinger_release(struct inode *inode,struct file *file)
{
	struct cdfingerfp_data *cdfingerfp = file->private_data;

	CDFINGER_DBG("enter\n");

	if (NULL == cdfingerfp)
	{
		CDFINGER_ERR("cdfingerfp is NULL\n");
		return -EIO;
	}

	file->private_data = NULL;

	CDFINGER_DBG("exit\n");

	return 0;
}

static void cdfinger_async_report(void)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;

	pm_wakeup_event(&cdfingerfp->cdfinger_dev->dev, 1000); // msec

	kill_fasync(&cdfingerfp->async_queue,SIGIO,POLL_IN);
}

static irqreturn_t cdfinger_eint_handler(int irq, void *dev_id)
{
	struct cdfingerfp_data *pdata = g_cdfingerfp_data;

	CDFINGER_DBG("irq = %d\n", irq);

	if (pdata->irq_enable_status == 1)
	{
		cdfinger_async_report();
	}

	return IRQ_HANDLED;
}

static int cdfinger_init_irq(struct cdfingerfp_data *pdata)
{
	int error = 0;

	if (isInit == 1)
		return 0;

	error = request_irq(gpio_to_irq(pdata->irq_num),cdfinger_eint_handler,IRQF_TRIGGER_RISING,"cdfinger_eint", NULL);
	if (error < 0)
	{
		CDFINGER_ERR("irq init err\n");
		return error;
	}
	enable_irq_wake(gpio_to_irq(pdata->irq_num));
	pdata->irq_enable_status = 1;
	isInit = 1;
	return error;
}

static void cdfinger_wake_lock(struct cdfingerfp_data *pdata, int arg)
{
	if (arg)
	{
		pm_stay_awake(&pdata->cdfinger_dev->dev);
	}
	else
	{
		pm_relax(&pdata->cdfinger_dev->dev);
		pm_wakeup_event(&pdata->cdfinger_dev->dev, 1000); // 1 sec
	}
}

static void cdfinger_enable_irq(struct cdfingerfp_data *pdata)
{
	if (pdata->irq_enable_status == 0)
	{
		enable_irq(gpio_to_irq(pdata->irq_num));
		pdata->irq_enable_status = 1;
	}
}

static void cdfinger_disable_irq(struct cdfingerfp_data *pdata)
{
	if (pdata->irq_enable_status == 1)
	{
		disable_irq(gpio_to_irq(pdata->irq_num));
		pdata->irq_enable_status = 0;
	}
}

static int cdfinger_irq_controller(struct cdfingerfp_data *pdata, int Onoff)
{
	if (isInit == 0)
	{
		CDFINGER_ERR("irq  not request!!!\n");
		return -1;
	}
	if (Onoff == 1)
	{
		cdfinger_enable_irq(pdata);
		return 0;
	}
	if (Onoff == 0)
	{
		cdfinger_disable_irq(pdata);
		return 0;
	}
	CDFINGER_ERR("irq  status parameter err %d !!!\n", Onoff);
	return -1;
}

static int cdfinger_report_key(struct cdfingerfp_data *cdfinger, unsigned long arg)
{
	key_report_t report;
	if ( copy_from_user(&report, (key_report_t *)arg, sizeof(key_report_t)) )
	{
		CDFINGER_ERR("%s err\n", __func__);
		return -1;
	}
	input_report_key(cdfinger->cdfinger_input, report.key, !!report.value);
	input_sync(cdfinger->cdfinger_input);

	return 0;
}

static long cdfinger_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	struct cdfingerfp_data *cdfinger = filp->private_data;

	mutex_lock(&cdfinger->buf_lock);
	switch (cmd) {
		case CDFINGER_INIT_GPIO:
			err = cdfinger_init_gpio(cdfinger);
			break;

		case CDFINGER_INIT_IRQ:
			err = cdfinger_init_irq(cdfinger);
#ifdef SUPPORT_FP_ID_DUAL
			cdfinger->chip_id = 0x88;
#endif // SUPPORT_FP_ID_DUAL
			break;

		case CDFINGER_RELEASE_DEVICE:
			cdfinger_free_gpio(cdfinger);
#ifdef SUPPORT_FP_ID_DUAL
			cdfinger->chip_id = 0x00;
#endif // SUPPORT_FP_ID_DUAL
			misc_deregister(cdfinger->miscdev);
			break;

		case CDFINGER_WAKE_LOCK:
			cdfinger_wake_lock(cdfinger,arg);
			break;

		case CDFINGER_POWER_ON:
			err = cdfinger_power_on(cdfinger);
			break;
		case CDFINGER_POWER_OFF:
		err = cdfinger_power_off(cdfinger);
			break;
		case CDFINGER_RESET:
			cdfinger_reset(cdfinger,1);
			break;

		case CDFINGER_INITERRUPT_MODE:
			isInKeyMode = 1;  // not key mode
			break;

		case CDFINGER_NEW_KEYMODE:
			isInKeyMode = 0;
			break;

		case CDFINGER_HW_RESET:
			cdfinger_reset(cdfinger,arg);
			break;

		case CDFINGER_GET_STATUS:
			err = screen_status;
			break;

		case CDFINGER_REPORT_KEY:
			err = cdfinger_report_key(cdfinger, arg);
			break;
		case CDFINGER_ENABLE_IRQ:
			err = cdfinger_irq_controller(cdfinger, arg);
			break;
		case CDFINGER_GETID:
#ifdef SUPPORT_FP_ID_DUAL
			err = cdfinger->chip_id;
#endif // SUPPORT_FP_ID_DUAL
			break;
		default:
			break;
	}
	mutex_unlock(&cdfinger->buf_lock);

	return err;
}

static const struct file_operations cdfinger_fops = {
	.owner	 = THIS_MODULE,
	.open	 = cdfinger_open,
	.unlocked_ioctl = cdfinger_ioctl,
	.release = cdfinger_release,
	.fasync  = cdfinger_async_fasync,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cdfinger_ioctl,
#endif
};

static struct miscdevice st_cdfinger_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &cdfinger_fops,
};

static int cdfinger_fb_notifier_callback(struct notifier_block* self,
										unsigned long event, void* data)
{
	struct fb_event* evdata = data;
	unsigned int blank;
	int retval = 0;

	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
		return 0;
	}

	blank = *(int*) evdata->data;

	switch (blank) {
		case FB_BLANK_UNBLANK:
			mutex_lock(&g_cdfingerfp_data->buf_lock);
			screen_status = 1;
			if (isInKeyMode == 0)
				cdfinger_async_report();
			mutex_unlock(&g_cdfingerfp_data->buf_lock);
			break;

		case FB_BLANK_POWERDOWN:
			mutex_lock(&g_cdfingerfp_data->buf_lock);
			screen_status = 0;
			if (isInKeyMode == 0)
				cdfinger_async_report();
			mutex_unlock(&g_cdfingerfp_data->buf_lock);
			break;

		default:
			break;
	}

	return retval;
}
#ifdef SUPPORT_FP_ID_DUAL
static int cdfinger_support_id(struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	if (cdfinger->fps_id_high == NULL) {
		cdfinger->fps_id_high = pinctrl_lookup_state(cdfinger->fps_pinctrl, "cdfinger_id_pin");

		if (IS_ERR(cdfinger->fps_id_high)) {
			CDFINGER_ERR("look up state err\n");
			return -1;
		}

		pinctrl_select_state(cdfinger->fps_pinctrl, cdfinger->fps_id_high);
	}

	if (!gpio_is_valid(cdfinger->id_num)) {
		CDFINGER_ERR("not valid irq gpio\n");
		return -EIO;
	}

	err = gpio_request(cdfinger->id_num, "cdfinger-id");
	if (err) {
		CDFINGER_ERR("Could not request id gpio. err = %d, retry once\n", err);

		gpio_free(cdfinger->id_num);

		err = gpio_request(cdfinger->id_num, "cdfinger-id");
		if (err) {
			CDFINGER_ERR("Could not request id gpio. return err = %d\n", err);
			return err;
		}
	}

	gpio_direction_input(cdfinger->id_num);
	udelay(10);

	err = gpio_get_value(cdfinger->id_num);
	gpio_free(cdfinger->id_num);

	return err;
}

static ssize_t get_fp_id (
		struct device* device,
		struct device_attribute* attribute,
		char* buffer)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	(void) device;
	(void) attribute;

	CDFINGER_DBG("fp_id = %d\n", cdfingerfp->fp_id);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", cdfingerfp->fp_id);
}

static ssize_t set_fp_id (
		struct device* device,
		struct device_attribute* attribute,
		const char* buffer,
		size_t count)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	(void) device;
	(void) attribute;

	if (*buffer == '1')
	{
		cdfingerfp->fp_id = 1;
	}
	else
	{
		cdfingerfp->fp_id = 0;
	}

	CDFINGER_DBG("fp_id = %d\n", cdfingerfp->fp_id);

	return count;
}

static ssize_t get_fp_id_real (
		struct device* device,
		struct device_attribute* attribute,
		char* buffer)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	int fp_id_real = cdfinger_support_id(cdfingerfp);
	(void) device;
	(void) attribute;

	CDFINGER_DBG("fp_id_real = %d\n", fp_id_real);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", fp_id_real);
}

static DEVICE_ATTR(fp_id, S_IRUGO | S_IWUSR, get_fp_id, set_fp_id);
static DEVICE_ATTR(fp_id_real, S_IRUGO | S_IWUSR, get_fp_id_real, NULL);

static struct attribute *attributes[] = {
	&dev_attr_fp_id.attr,
	&dev_attr_fp_id_real.attr,
	NULL,
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};
#endif // SUPPORT_FP_ID_DUAL

static int cdfinger_probe(struct platform_device *pdev)
{
	struct cdfingerfp_data *cdfingerdev= NULL;
	int status = -ENODEV;
	int i;

	CDFINGER_DBG("enter\n");

	cdfingerdev = kzalloc(sizeof(struct cdfingerfp_data), GFP_KERNEL);
	if (cdfingerdev == NULL) {
		CDFINGER_ERR("kzalloc failed!\n");

		return -ENOMEM;
	}

	cdfingerdev->cdfinger_dev = pdev;

	status = cdfinger_parse_dts(&cdfingerdev->cdfinger_dev->dev, cdfingerdev);
	if (status) {
		CDFINGER_ERR("parse dts err %d\n",status);
		goto free_alloc;
	}

#ifdef SUPPORT_FP_ID_DUAL
	cdfingerdev->fp_id = -1;

	status = cdfinger_support_id(cdfingerdev);

	if (status != FP_ID_VALUE) {
		CDFINGER_ERR("fp id value error %d\n", status);
		status = -1;
		goto free_alloc;
	}

	CDFINGER_DBG("fp id value %d\n", status);

	cdfingerdev->fp_id = status;
#endif // SUPPORT_FP_ID_DUAL

#ifdef POWER_REGULATOR
	cdfingerdev->vdd = regulator_get(&cdfingerdev->cdfinger_dev->dev, "vdd");
#endif

	status = misc_register(&st_cdfinger_dev);
	if (status) {
		CDFINGER_ERR("misc register err%d\n", status);
		goto free_regulator;
	}

	cdfingerdev->miscdev = &st_cdfinger_dev;
	mutex_init(&cdfingerdev->buf_lock);

	device_init_wakeup(&cdfingerdev->cdfinger_dev->dev, true);

	cdfingerdev->cdfinger_input = input_allocate_device();
	if (!cdfingerdev->cdfinger_input) {
		CDFINGER_ERR("create cdfinger_input failed!\n");
		goto release_wakelock;
	}

	for (i = 0; i < ARRAY_SIZE(maps); i++) {
		input_set_capability(cdfingerdev->cdfinger_input, maps[i].type, maps[i].code);
	}

	cdfingerdev->cdfinger_input->name = INPUT_DEVICE_NAME;
	cdfingerdev->cdfinger_input->dev.init_name = "lge_fingerprint";
	status = input_register_device(cdfingerdev->cdfinger_input);
	if (status) {
		CDFINGER_ERR("register input device err = %d\n", status);
		cdfingerdev->cdfinger_input = NULL;
		goto free_input;
	}

#ifdef SUPPORT_FP_ID_DUAL
	status = sysfs_create_group(&cdfingerdev->cdfinger_input->dev.kobj, &attribute_group);

	if (status) {
		CDFINGER_ERR("sysfs create group err %d\n", status);
		goto unregister_input;
	}
#endif // SUPPORT_FP_ID_DUAL

	cdfingerdev->notifier.notifier_call = cdfinger_fb_notifier_callback;
	status = fb_register_client(&cdfingerdev->notifier);

	if (status) {
		CDFINGER_ERR("fb register client err %d\n", status);
		goto unregister_group;
	}

	g_cdfingerfp_data = cdfingerdev;

	CDFINGER_DBG("exit\n");

	return 0;

unregister_group:
#ifdef SUPPORT_FP_ID_DUAL
	sysfs_remove_group(&cdfingerdev->cdfinger_input->dev.kobj, &attribute_group);
#endif

#ifdef SUPPORT_FP_ID_DUAL
unregister_input:
	input_unregister_device(cdfingerdev->cdfinger_input); // remove fp_id node
#endif

free_input:
	input_free_device(cdfingerdev->cdfinger_input);

release_wakelock:
	device_init_wakeup(&cdfingerdev->cdfinger_dev->dev, false);

//release_mutex:
	mutex_destroy(&cdfingerdev->buf_lock);

//unregister_misc:
	misc_deregister(&st_cdfinger_dev);

free_regulator:
#ifdef POWER_REGULATOR
	regulator_put(cdfingerdev->vdd);
#endif

free_alloc:
	kfree(cdfingerdev);
	CDFINGER_ERR("error status = %d\n", status);
	return status;
}


static const struct of_device_id cdfinger_of_match[] = {
	{ .compatible = "cdfinger,fps998e", },
	{ .compatible = "cdfinger,fingerprint", },
	{},
};

static const struct platform_device_id cdfinger_id[] = {
	{"cdfinger_fp", 0},
	{}
};

static struct platform_driver cdfinger_driver = {
	.driver = {
		.name = "cdfinger_fp",
		.of_match_table = cdfinger_of_match,
	},
	.id_table = cdfinger_id,
	.probe = cdfinger_probe,
};

static int __init cdfinger_fp_init(void)
{
#ifdef SUPPORT_ONE_BINARY
	{
		int is_support = lge_get_fingerprint_hwinfo();

		if (!is_support) {
			printk(KERN_WARNING "[cdfinger] %s: doesn't support fingerprint, skip register\n", __func__);

			return 0;
		}

		printk(KERN_INFO "[cdfinger] %s: support fingerprint\n", __func__);
	}
#endif // SUPPORT_ONE_BINARY

	return platform_driver_register(&cdfinger_driver);
}

static void __exit cdfinger_fp_exit(void)
{
#ifdef SUPPORT_ONE_BINARY
	{
		int is_support = lge_get_fingerprint_hwinfo();

		if (!is_support) {
			printk(KERN_WARNING "[cdfinger] %s: doesn't support fingerprint, skip register\n", __func__);

			return ;
		}

		printk(KERN_INFO "[cdfinger] %s: support fingerprint\n", __func__);
	}
#endif // SUPPORT_ONE_BINARY

	platform_driver_unregister(&cdfinger_driver);
}

late_initcall(cdfinger_fp_init);
module_exit(cdfinger_fp_exit);

MODULE_DESCRIPTION("cdfinger Driver");
MODULE_AUTHOR("yangzhipeng@cdfinger.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cdfinger");
