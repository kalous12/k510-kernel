/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _VI_WRAP_DRV_H_
#define _VI_WRAP_DRV_H_

typedef struct _VI_WRAP_RESET_CTL_S
{
    unsigned int    csi_00_rst_en;
    unsigned int    csi_01_rst_en;
    unsigned int    csi_02_rst_en;
    unsigned int    csi_10_rst_en;
	
    unsigned int    csi_11_rst_en;
    unsigned int    csi_12_rst_en;
    unsigned int    dvp_0_rst_en;
    unsigned int    dvp_1_rst_en;
	
    unsigned int    axi_wr_ch_rst_en;
    unsigned int    axi_rd_ch_rst_en;
}VI_WRAP_RESET_CTL_S;

typedef struct _VI_WRAP_CSI_MODE_CTL_S
{
    unsigned int    mipi_csi_0_mode;
    unsigned int    mipi_csi_1_mode;
    unsigned int    mipi_dphy_mode;
    unsigned int    mipi_csi01_dbg_sel;
	
    unsigned int    isp_4k_clk_sel;
    unsigned int    isp_2k_clk_sel;
    unsigned int    isp_r_2k_clk_sel;
    unsigned int    tof_clk_sel;
	
    unsigned int    csi_0_sony_dol_en;
    unsigned int    dvp_clk_0_mode;
} VI_WRAP_CSI_MODE_CTL_S;

typedef struct _VI_WRAP_ISP_CH_SEL_S
{
    unsigned int    isp_4k_l_ch_sel;
    unsigned int    isp_4k_m_ch_sel;
    unsigned int    isp_4k_s_ch_sel;
    unsigned int    isp_2k_l_ch_sel;
    unsigned int    isp_2k_m_ch_sel;
    unsigned int    isp_2k_s_ch_sel;
    unsigned int    isp_r_2k_ch_sel;
    unsigned int    isp_3d_ch_sel;
} VI_WRAP_ISP_CH_SEL_S;

typedef struct _VI_WRAP_CLOCK_CTL_S
{
    unsigned int    csi_00_pixel_clk_en;
    unsigned int    csi_01_pixel_clk_en;
    unsigned int    csi_02_pixel_clk_en;
    unsigned int    csi_10_pixel_clk_en;
    
    unsigned int    csi_11_pixel_clk_en;
    unsigned int    csi_12_pixel_clk_en;
    unsigned int    dvp_0_pixel_clk_en;
    unsigned int    dvp_1_pixel_clk_en;
    
    unsigned int    csi_00_dma_clk_en;
    unsigned int    csi_01_dma_clk_en;
    unsigned int    csi_02_dma_clk_en;
    unsigned int    csi_10_dma_clk_en;
    
    unsigned int    csi_11_dma_clk_en;
    unsigned int    csi_12_dma_clk_en;
    unsigned int    dvp_0_dma_clk_en;
    unsigned int    dvp_1_dma_clk_en;
    
    unsigned int    axi_wr_ch_clk_en;
    unsigned int    axi_rd_ch_clk_en;
}VI_WRAP_CLOCK_CTL_S;

typedef struct _VI_WRAP_DMA_ARB_MODE_S
{
    unsigned int   wr_dma_arb_mode;
    unsigned int   rd_dma_arb_mode;
} VI_WRAP_DMA_ARB_MODE_S;

typedef struct _VI_WRAP_DMA_WR_WEIGHT_0_S
{
    unsigned int    vi_dma_ch0_wr_weight;
    unsigned int    vi_dma_ch1_wr_weight;
    unsigned int    vi_dma_ch2_wr_weight;
    unsigned int    vi_dma_ch3_wr_weight;
} VI_WRAP_DMA_WR_WEIGHT_0_S;

typedef struct _VI_WRAP_DMA_WR_WEIGHT_1_S
{
    unsigned int    vi_dma_ch4_wr_weight;
    unsigned int    vi_dma_ch5_wr_weight;
    unsigned int    vi_dma_ch6_wr_weight;
    unsigned int    vi_dma_ch7_wr_weight;
} VI_WRAP_DMA_WR_WEIGHT_1_S;

typedef struct _VI_WRAP_DMA_RD_WEIGHT_0_S
{
    unsigned int    vi_dma_ch0_rd_weight;
    unsigned int    vi_dma_ch1_rd_weight;
    unsigned int    vi_dma_ch2_rd_weight;
    unsigned int    vi_dma_ch3_rd_weight;
} VI_WRAP_DMA_RD_WEIGHT_0_S;

typedef struct _VI_WRAP_DMA_RD_WEIGHT_1_S
{
    unsigned int    vi_dma_ch4_rd_weight;
    unsigned int    vi_dma_ch5_rd_weight;
    unsigned int    vi_dma_ch6_rd_weight;
    unsigned int    vi_dma_ch7_rd_weight;
}VI_WRAP_DMA_RD_WEIGHT_1_S;

typedef struct _VI_WRAP_DMA_WR_PRIORITY_S
{
    unsigned int    vi_dma_ch0_wr_priority;
    unsigned int    vi_dma_ch1_wr_priority;
    unsigned int    vi_dma_ch2_wr_priority;
    unsigned int    vi_dma_ch3_wr_priority;

    unsigned int    vi_dma_ch4_wr_priority;
    unsigned int    vi_dma_ch5_wr_priority;
    unsigned int    vi_dma_ch6_wr_priority;
    unsigned int    vi_dma_ch7_wr_priority;
} VI_WRAP_DMA_WR_PRIORITY_S;

typedef struct _VI_WRAP_DMA_RD_PRIORITY_S
{
    unsigned int    vi_dma_ch0_rd_priority;
    unsigned int    vi_dma_ch1_rd_priority;
    unsigned int    vi_dma_ch2_rd_priority;
    unsigned int    vi_dma_ch3_rd_priority;

    unsigned int    vi_dma_ch4_rd_priority;
    unsigned int    vi_dma_ch5_rd_priority;
    unsigned int    vi_dma_ch6_rd_priority;
    unsigned int    vi_dma_ch7_rd_priority;
} VI_WRAP_DMA_RD_PRIORITY_S;

typedef struct _VI_WRAP_DMA_WR_CH_ID_S
{
    unsigned int    vi_dma_wr_ch0_id;
    unsigned int    vi_dma_wr_ch1_id;
    unsigned int    vi_dma_wr_ch2_id;
    unsigned int    vi_dma_wr_ch3_id;

    unsigned int    vi_dma_wr_ch4_id;
    unsigned int    vi_dma_wr_ch5_id;
    unsigned int    vi_dma_wr_ch6_id;
    unsigned int    vi_dma_wr_ch7_id;
} VI_WRAP_DMA_WR_CH_ID_S;

typedef struct _VI_WRAP_DMA_RD_CH_ID_S
{
    unsigned int   vi_dma_rd_ch0_id;
    unsigned int   vi_dma_rd_ch1_id;
    unsigned int   vi_dma_rd_ch2_id;
    unsigned int   vi_dma_rd_ch3_id;
    unsigned int   vi_dma_rd_ch4_id;
    unsigned int   vi_dma_rd_ch5_id;
    unsigned int   vi_dma_rd_ch6_id;
    unsigned int   vi_dma_rd_ch7_id;
} VI_WRAP_DMA_RD_CH_ID_S;

typedef struct _VI_WRAP_DMA_ATTR_S 
{
    VI_WRAP_DMA_ARB_MODE_S          stDmaArbMode;
    VI_WRAP_DMA_WR_WEIGHT_0_S       stWrChWeight0;
    VI_WRAP_DMA_WR_WEIGHT_1_S       stWrChWeight1;
    VI_WRAP_DMA_RD_WEIGHT_0_S       stRdChWeight0;
    VI_WRAP_DMA_RD_WEIGHT_1_S       stRdChWeight1;
    VI_WRAP_DMA_WR_PRIORITY_S       stWrChPriority;
    VI_WRAP_DMA_RD_PRIORITY_S       stRdChPriority;
    VI_WRAP_DMA_WR_CH_ID_S          stWriteChId;
    VI_WRAP_DMA_RD_CH_ID_S          stReadChId; 
} VI_WRAP_DMA_ATTR_S;

typedef struct _VI_WRAP_INT_EN_S 
{
     unsigned int  csi_0_host_int_en;    
     unsigned int  csi_0_host_err_int_en;
     unsigned int  csi_1_host_int_en;    
     unsigned int  csi_1_host_err_int_en;
     unsigned int  csi_0_ctrl_0_int_en;  
     unsigned int  csi_0_ctrl_1_int_en;  
     unsigned int  csi_0_ctrl_2_int_en;  
     unsigned int  dvp_0_ctrl_int_en;    
     unsigned int  csi_1_ctrl_0_int_en;  
     unsigned int  csi_1_ctrl_1_int_en;  
     unsigned int  csi_1_ctrl_2_int_en;  
     unsigned int  dvp_1_ctrl_int_en;    
}VI_WRAP_INT_EN_S; 

typedef struct _VI_WRAP_CFG_DONE_S 
{
     unsigned int vi_wrap_config_done;
     unsigned int vi_wrap_wp_clr;
}VI_WRAP_CFG_DONE_S;

typedef struct _VI_WRAP_ATTR_S 
{
     VI_WRAP_RESET_CTL_S        stRestCtl;
     VI_WRAP_CSI_MODE_CTL_S     stWrapCsiCtl;
     VI_WRAP_ISP_CH_SEL_S       stWrapIspChSel;
     VI_WRAP_CLOCK_CTL_S        stWarpClockCtl;
     VI_WRAP_DMA_ATTR_S         stWrapDmaAttr;
     VI_WRAP_CFG_DONE_S         stWrapCfgDone;
} VI_WRAP_ATTR_S;
/*
*
*/
void VI_DRV_WRAP_SetRst(struct k510_isp_device *isp,VI_WRAP_RESET_CTL_S *pstRstCtl);
int VI_DRV_WRAP_SetCtlMode(struct k510_isp_device *isp,VI_WRAP_CSI_MODE_CTL_S *pstCsiModeCtl);
int VI_DRV_WRAP_SetIspChSel(struct k510_isp_device *isp,VI_WRAP_ISP_CH_SEL_S *pstIspChSel);
int VI_DRV_WRAP_SetClockCtl(struct k510_isp_device *isp,VI_WRAP_CLOCK_CTL_S *pstClkCtl);
int VI_DRV_WRAP_SetDmaAttr(struct k510_isp_device *isp,VI_WRAP_DMA_ATTR_S *pstDmaAttr);
int VI_DRV_WRAP_SetCfgDone(struct k510_isp_device *isp,VI_WRAP_CFG_DONE_S *pstWrapCfgDone);
int VI_DRV_WRAP_SetIntEn(struct k510_isp_device *isp,VI_WRAP_INT_EN_S *pstIntEn);
#endif /*_VI_WRAP_DRV_H_*/