#include <linux/delay.h>
#include "mdss_dsi.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
#define EXT_DSV_PRIVILEGED
#include <linux/mfd/external_dsv.h>
#endif

extern bool flag_panel_deep_sleep_ctrl;
extern bool flag_panel_deep_sleep_status;

void boe_ft8006p_panel_enter_deep_sleep(void)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	if (flag_panel_deep_sleep_ctrl) {
		pdata = lge_mdss_dsi_get_ctrl_pdata();
		if (pdata == NULL)
			return;

		lge_extra_gpio_set_value(pdata, "touch_reset", 0);
		usleep_range(2000, 2000);

		gpio_set_value((pdata->rst_gpio), 0);
		usleep_range(2000, 2000);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		ext_dsv_mode_change(POWER_OFF);
		lge_extra_gpio_set_value(pdata, "dsv_vsn_en", 0);
		usleep_range(5000, 5000);
		lge_extra_gpio_set_value(pdata, "dsv_vsp_en", 0);
		usleep_range(5000, 5000);
#endif
		lge_extra_gpio_set_value(pdata, "vddio", 0);
		usleep_range(1000, 1000);

		flag_panel_deep_sleep_status = true;
		pr_info("%s done \n", __func__);
	}
}

void boe_ft8006p_panel_exit_deep_sleep(void)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	if (flag_panel_deep_sleep_ctrl) {
		pdata = lge_mdss_dsi_get_ctrl_pdata();
		if (pdata == NULL)
			return;

		mdss_dsi_clk_ctrl(pdata, pdata->dsi_clk_handle, MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
		mdss_dsi_sw_reset(pdata, true);

		lge_extra_gpio_set_value(pdata, "vddio", 1);
		usleep_range(12000, 12000);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		lge_extra_gpio_set_value(pdata, "dsv_vsp_en", 1);
		usleep_range(12000, 12000);
		lge_extra_gpio_set_value(pdata, "dsv_vsn_en", 1);
		usleep_range(2000, 2000);
		ext_dsv_mode_change(POWER_ON);
#endif
		usleep_range(2000, 2000);

		gpio_set_value((pdata->rst_gpio), 1);
		usleep_range(1000, 1000);

		lge_extra_gpio_set_value(pdata, "touch_reset", 1);
		usleep_range(6000, 6000);

		lge_mdss_dsi_panel_extra_cmds_send(pdata, "lpwg-on");

		mdss_dsi_clk_ctrl(pdata, pdata->dsi_clk_handle, MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);

		flag_panel_deep_sleep_status = false;

		pr_info("%s done \n", __func__);
	}
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
extern int mdss_dsi_set_clk_src(struct mdss_dsi_ctrl_pdata *ctrl);
void boe_ft8006p_mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);
	int ret = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	ret += mdss_dsi_set_clk_src(ctrl_pdata);
	ret += mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	if (ret) {
		pr_err("%s: could fail to set LP11\n", __func__);
	}
	usleep_range(5000, 5000);

	lge_extra_gpio_set_value(ctrl_pdata, "touch_reset", 0);
	usleep_range(2000, 2000);

	gpio_set_value((ctrl_pdata->rst_gpio), 0);

	ret += mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
		MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
	if (ret) {
		pr_err("%s: could fail to set LP00\n", __func__);
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	ext_dsv_mode_change(POWER_OFF);
	lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsn_en", 0);
	usleep_range(5000, 5000);
	lge_extra_gpio_set_value(ctrl_pdata, "dsv_vsp_en", 0);
	usleep_range(5000, 5000);
#endif

	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	usleep_range(2000, 2000);

	pr_info("%s: panel shutdown done \n", __func__);

	return;
}
#endif
