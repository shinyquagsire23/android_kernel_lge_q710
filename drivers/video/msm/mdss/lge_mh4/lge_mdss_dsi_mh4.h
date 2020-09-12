#ifndef LGE_MDSS_DSI_MH4
#define LGE_MDSS_DSI_MH4
void ili9881h_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
void ili9881h_panel_enter_deep_sleep(void);
void ili9881h_panel_exit_deep_sleep(void);
void cpt_ft8006p_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
void cpt_ft8006p_panel_enter_deep_sleep(void);
void cpt_ft8006p_panel_exit_deep_sleep(void);
void boe_ft8006p_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
void boe_ft8006p_panel_enter_deep_sleep(void);
void boe_ft8006p_panel_exit_deep_sleep(void);
#endif
