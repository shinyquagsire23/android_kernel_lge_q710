#include <linux/delay.h>
#include "mdss_dsi.h"

#include "lge/mfts_mode.h"

#include <soc/qcom/lge/board_lge.h>
#include "lge_mdss_dsi_mh4.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
#define EXT_DSV_PRIVILEGED
#include <linux/mfd/external_dsv.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY)
#include <linux/msm_lcd_recovery.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
#include <soc/qcom/lge/board_lge.h>
#endif

#include <linux/input/lge_touch_notify.h>
#include <linux/msm_lcd_power_mode.h>
enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

/* Touch LPWG Status */
static unsigned int pre_panel_mode = LCD_MODE_STOP;
static unsigned int cur_panel_mode = LCD_MODE_STOP;

#if defined(CONFIG_LGE_DISPLAY_RECOVERY)
int esd_detected = 0;
static int panel_recovery_flag = 0;
#endif

bool flag_panel_deep_sleep_ctrl = false;
bool flag_panel_deep_sleep_status = false;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
extern int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

void lge_panel_enter_deep_sleep(void)
{
	switch (lge_get_panel_type()) {
		case BOE_INCELL_ILI9881H:
			ili9881h_panel_enter_deep_sleep();
			break;
		case CPT_INCELL_FT8006P:
			cpt_ft8006p_panel_enter_deep_sleep();
			break;
		case BOE_INCELL_FT8006P:
			boe_ft8006p_panel_enter_deep_sleep();
			break;
		default:
			break;
	}
}

void lge_panel_exit_deep_sleep(void)
{
	usleep_range(10000, 10000);
	switch (lge_get_panel_type()) {
		case BOE_INCELL_ILI9881H:
			ili9881h_panel_exit_deep_sleep();
			break;
		case CPT_INCELL_FT8006P:
			cpt_ft8006p_panel_exit_deep_sleep();
			break;
		case BOE_INCELL_FT8006P:
			boe_ft8006p_panel_exit_deep_sleep();
			break;
		default:
			break;
	}
}

void lge_panel_set_power_mode(int mode)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	pr_info("%s start, mode = %d \n", __func__, mode);

	pdata = lge_mdss_dsi_get_ctrl_pdata();
	switch (mode){
		case DEEP_SLEEP_ENTER:
			lge_panel_enter_deep_sleep();
			break;
		case DEEP_SLEEP_EXIT:
			lge_panel_exit_deep_sleep();
			break;
		default :
			break;
	}

	pr_info("%s done \n", __func__);
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_info("%s: + enable = %d (override: mh4)\n", __func__, enable);

	if (enable) {
		if (!pinfo->cont_splash_enabled) {
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}
			if(lge_get_panel_type() != CPT_INCELL_FT8006P)
				touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			usleep_range(2000, 2000);

			gpio_set_value((ctrl_pdata->rst_gpio), enable);
			usleep_range(2000, 2000);
		}
	}
	pr_info("%s: -\n", __func__);

	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s: + (override: mh4)\n", __func__);


	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
		usleep_range(5000, 5000);
		if(lge_get_panel_type()== BOE_INCELL_ILI9881H || lge_get_panel_type()== CPT_INCELL_FT8006P){
		usleep_range(7000, 7000);
		}
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsp_en", 1);
		usleep_range(5000, 5000);
		if(lge_get_panel_type()== CPT_INCELL_FT8006P)
			usleep_range(7000, 7000);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsn_en", 1);
		usleep_range(2000, 2000);
		ext_dsv_mode_change(POWER_ON);
#endif
	}

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret) {
		pr_err("%s: failed to enable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");

		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("%s: Panel reset failed. rc=%d\n",
					__func__, ret);
	}

	pr_info("%s: -\n", __func__);
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s: + (override: mh4)\n", __func__);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
	if (!ctrl_pdata->lge_extra.lp11_off) {
		ret = mdss_dsi_panel_reset(pdata, 0);
	}
#else
	ret = mdss_dsi_panel_reset(pdata, 0);
#endif

	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);

	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	if (lge_mdss_dsi_panel_power_seq_all()) {
		ext_dsv_mode_change(POWER_OFF);
		lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsn_en", 0);
		usleep_range(2000, 2000);

		if(lge_get_panel_type()== CPT_INCELL_FT8006P)
			usleep_range(3000, 3000);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsp_en", 0);
		usleep_range(2000, 2000);

		if(lge_get_panel_type()== CPT_INCELL_FT8006P)
			usleep_range(3000, 3000);
	}
#endif
	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
		usleep_range(2000, 2000);
	}

	pr_info("%s: -\n", __func__);
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
extern int mdss_dsi_set_clk_src(struct mdss_dsi_ctrl_pdata *ctrl);
extern int mdss_dsi_clk_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, void *clk_handle,
 enum mdss_dsi_clk_type clk_type, enum mdss_dsi_clk_state clk_state);

void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	switch (lge_get_panel_type()) {
		case BOE_INCELL_ILI9881H:
			ili9881h_mdss_dsi_ctrl_shutdown(pdev);
			break;
		case CPT_INCELL_FT8006P:
			cpt_ft8006p_mdss_dsi_ctrl_shutdown(pdev);
			break;
		case BOE_INCELL_FT8006P:
			boe_ft8006p_mdss_dsi_ctrl_shutdown(pdev);
			break;
		default:
			break;
	}
}
#endif


int lge_mdss_dsi_pre_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	switch (event) {
	default:
		pr_info("%s: nothing to do about this event=%d\n", __func__, event);
	}
	return rc;
}

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	switch (event) {
	case MDSS_EVENT_RESET:
		flag_panel_deep_sleep_ctrl = false;
		if (flag_panel_deep_sleep_status) {
				lge_set_panel_recovery_flag(true);
		}
		break;
	case MDSS_EVENT_UNBLANK:
		if (lge_mdss_dsi_panel_power_seq_all()) {
			lge_set_panel_recovery_flag(false);
		}
		pr_info("%s: event=MDSS_EVENT_UNBLANK\n", __func__);
		break;
	case MDSS_EVENT_POST_PANEL_ON:
		if(lge_get_panel_type() == CPT_INCELL_FT8006P)
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);

		cur_panel_mode = LCD_MODE_U3;
		pr_info("%s: event=MDSS_EVENT_PANEL_ON panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);

		if (flag_panel_deep_sleep_status) {
				lge_set_panel_recovery_flag(false);
				flag_panel_deep_sleep_status = false;
		}
		break;
	case MDSS_EVENT_BLANK:
		break;
	case MDSS_EVENT_PANEL_OFF:
		cur_panel_mode = LCD_MODE_U0;
		pr_info("%s: event=MDSS_EVENT_PANEL_OFF panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);
		flag_panel_deep_sleep_ctrl = true;
		break;
	default:
		pr_info("%s: nothing to do about this event=%d\n", __func__, event);
	}

	if (pre_panel_mode != cur_panel_mode) {
		rc = touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
		pre_panel_mode = cur_panel_mode;
	}

	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
int lge_get_lpwg_on_event(void)
{
		return MDSS_EVENT_MAX;
}
int lge_get_lpwg_off_event(void)
{
		return MDSS_EVENT_MAX;
}
#endif

#if defined(CONFIG_LGE_DISPLAY_RECOVERY)

int lge_get_panel_recovery_flag()
{
	pr_info("%s: flag=%d", __func__, panel_recovery_flag);
	return panel_recovery_flag;
}

void lge_set_panel_recovery_flag(int flag)
{
	pr_info("%s: flag=%d", __func__, flag);
	panel_recovery_flag = flag;
}

int lge_mdss_report_touchintpin_keep_low(void)
{
	pr_info("%s : D-IC is in abnormal status", __func__);
	lge_mdss_report_panel_dead(PANEL_HW_RESET);

	return 0;
}

int lge_mdss_dsi_panel_power_seq_all()
{
	int ret = 0;
#if IS_ENABLED(CONFIG_LGE_DISPLAY_MFTS)
	if (lge_get_display_power_ctrl())
		ret = 1;
#endif

	if (lge_get_panel_recovery_flag())
		ret = 1;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO)
		ret = 1;
#endif

	return ret;
}

EXPORT_SYMBOL(lge_mdss_report_touchintpin_keep_low);
#endif
