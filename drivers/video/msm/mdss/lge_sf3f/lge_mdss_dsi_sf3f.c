#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <soc/qcom/lge/board_lge.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "lge/lge_comfort_view.h"
#endif
#include "lge_mdss_dsi_sf3f.h"


#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			ret = sf3f_lgd_td4310_mdss_dsi_panel_power_on(pdata);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			ret = sf3f_lgd_sw49105_mdss_dsi_panel_power_on(pdata);
			break;
#endif
		default:
			break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			ret = sf3f_lgd_td4310_mdss_dsi_panel_power_off(pdata);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			ret = sf3f_lgd_sw49105_mdss_dsi_panel_power_off(pdata);
			break;
#endif
		default:
			break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
/*
 * mdss_dsi_request_gpios() should be defined in each panel file
 */
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
        int rc = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			rc = sf3f_lgd_td4310_mdss_dsi_panel_reset(pdata, enable);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			rc = sf3f_lgd_sw49105_mdss_dsi_panel_reset(pdata, enable);
			break;
#endif
		default:
			break;
	}

        return rc;
}
#endif


#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			sf3f_lgd_td4310_mdss_dsi_ctrl_shutdown(pdev);
		break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			sf3f_lgd_sw49105_mdss_dsi_ctrl_shutdown(pdev);
		break;
#endif
		default:
			break;
	}
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

static struct dsi_panel_cmds comfort_view_cmds[COMFORT_VIEW_STEP_NUMS];
static char *comfort_view_step_cmds[] = {
	"lge,comfort-view-cmds-off",
	"lge,comfort-view-cmds-step1",
	"lge,comfort-view-cmds-step2",
	"lge,comfort-view-cmds-step3",
	"lge,comfort-view-cmds-step4",
	"lge,comfort-view-cmds-step5",
	"lge,comfort-view-cmds-step6",
	"lge,comfort-view-cmds-step7",
	"lge,comfort-view-cmds-step8",
	"lge,comfort-view-cmds-step9",
	"lge,comfort-view-cmds-step10",
};

int lge_mdss_dsi_parse_comfort_view_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;

	for (i = 0 ; i < COMFORT_VIEW_STEP_NUMS ; i++) {
		mdss_dsi_parse_dcs_cmds(np, &comfort_view_cmds[i],
			comfort_view_step_cmds[i], "lge,comfort-view-cmds-state");
	}
	return 0;
}

static bool change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	if(comfort_view_cmds[new_mode].cmd_cnt) {
		pr_info("%s: sending comfort mode commands [%d]\n", __func__, new_mode);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
		mdss_dsi_panel_cmds_send(ctrl, &comfort_view_cmds[new_mode], CMD_REQ_COMMIT);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	}
	return true;
}

bool lge_change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{
	if (old_mode == new_mode) {
		pr_info("%s: same mode [%d]\n", __func__, new_mode);
		return true;
	}

	return change_comfort_view(ctrl, new_mode);
}

int lge_mdss_dsi_panel_send_post_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, int cur_mode)
{
	if (cur_mode != 0)
		change_comfort_view(ctrl, cur_mode);
	return 0;
}
#endif
