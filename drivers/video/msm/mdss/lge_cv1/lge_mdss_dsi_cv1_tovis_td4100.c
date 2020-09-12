#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/mfts_mode.h"
#include <linux/mfd/dw8768.h>
#include <soc/qcom/lge/board_lge.h>

#if defined(CONFIG_LGE_DISPLAY_RECOVERY)
#include <linux/msm_lcd_recovery.h>

int esd_detected = 0;
static int panel_recovery_flag = 0;
#endif

#include <linux/input/lge_touch_notify.h>
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

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int tovis_td4100_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if (lge_mdss_dsi_panel_power_seq_all()) {
		if (lge_get_display_power_ctrl()) {
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 1);
			pr_info("mfts-power gpio set high\n");
		}
		ret = msm_dss_enable_vreg(
				ctrl_pdata->panel_power_data.vreg_config,
				ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret) {
			pr_err("%s: failed to disable vregs for %s\n",
					__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}
	}
	else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}

end:
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int tovis_td4100_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (lge_mdss_dsi_panel_power_seq_all()) {
		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}
		if (lge_get_display_power_ctrl()) {
			/* 1v8 Active Low at MFTS JIG */
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 0);
			pr_info("mfts-power gpio set low\n");
		}
	}
	else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
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

	return ret;
}
#endif

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}

	rc = lge_extra_gpio_request(ctrl_pdata, "dsv-ena");
	if (rc) {
		pr_err("request dsv-ena gpio failed, rc=%d\n",
			rc);
		goto ena_gpio_err;
	}

	return rc;
ena_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int tovis_td4100_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
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
	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
			__func__, __LINE__);
		return rc;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	pr_info("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			pr_info("%s: panel reset\n", __func__);
			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
					pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n",
						__func__);
					goto exit;
				}
			}

			if (lge_mdss_dsi_panel_power_seq_all()) {
				pr_info("%s: turn panel power on\n", __func__);
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				mdelay(10);
				gpio_set_value((ctrl_pdata->rst_gpio), 1);
				mdelay(20);
				lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
				mdelay(5);
				dw8768_register_set(0x05, 0x0E);
				mdelay(10);
				dw8768_register_set(0x05, 0x0F);
				mdelay(130);
			}
			else{
				pr_info("%s: skip panel power control\n", __func__);
				for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
					gpio_set_value((ctrl_pdata->rst_gpio), pdata->panel_info.rst_seq[i]);

					if (pdata->panel_info.rst_seq[++i])
						usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
				}
			}
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
			pr_info("%s: turn panel power off\n", __func__);
			lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0); //DSV low
			mdelay(10);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			mdelay(10);
		}
		lge_extra_gpio_free(ctrl_pdata, "dsv-ena");
		gpio_free(ctrl_pdata->rst_gpio);
	}

	pr_info("%s: -\n", __func__);
exit:
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void tovis_td4100_mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}


	lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);	//DSV low
	mdelay(10);

	pr_info("%s: reset to low\n", __func__);
	gpio_set_value((ctrl_pdata->rst_gpio), 0);		//Reset low
	mdelay(10);

	if (lge_get_display_power_ctrl()) {
		lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 1);
		pr_err("mfts-power gpio set\n");
	}

	pr_info("%s: turn panel shutdown\n", __func__);

	return;

}
#endif

int tovis_td4100_mdss_dsi_pre_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
	case MDSS_EVENT_LINK_READY:
		rc = touch_notifier_call_chain(LCD_EVENT_LCD_UNBLANK, (void *)&event);
		break;
	case MDSS_EVENT_BLANK:
		rc = touch_notifier_call_chain(LCD_EVENT_LCD_BLANK, (void *)&event);
		break;
	default:
		break;
	}

	return rc;
}

int tovis_td4100_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
	case MDSS_EVENT_RESET:
		break;
	case MDSS_EVENT_UNBLANK:
		if (lge_mdss_dsi_panel_power_seq_all()) {
			lge_set_panel_recovery_flag(false);
		}
		break;
	case MDSS_EVENT_POST_PANEL_ON:
		cur_panel_mode = LCD_MODE_U3;
		pr_info("%s: event=MDSS_EVENT_POST_PANEL_ON panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);
		break;
	case MDSS_EVENT_BLANK:
		break;
	case MDSS_EVENT_PANEL_OFF:
		cur_panel_mode = LCD_MODE_U0;
		pr_info("%s: event=MDSS_EVENT_PANEL_OFF panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);
		break;
	default:
		break;
	}

	if (pre_panel_mode != cur_panel_mode) {
		pre_panel_mode = cur_panel_mode;
	}

	return rc;
}
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
#endif
