/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef _ISP_WRAP_DRV_H_
#define _ISP_WRAP_DRV_H_

#include "isp_wrap_reg.h"

#define ISP_DMA_CH_NUM 16

typedef struct _ISP_WRAP_RST_CTL_S
{
    unsigned int sw_3dnr_rst ;    
    unsigned int wdr_2_frame_rst ;
    unsigned int wdr_3_frame_rst ;
    unsigned int ldc_rst ;        
    unsigned int main_out_rst ;   
    unsigned int ds0_out_rst ;    
    unsigned int ds1_out_rst ;    
    unsigned int ds2_out_rst ;    
    unsigned int wrap_com_rst ;   
    unsigned int wrap_cfg_rst ;   
    unsigned int core_com_rst ;   
    unsigned int core_cfg_rst ;   
    unsigned int axi_wr_ch_rst ;  
    unsigned int axi_rd_ch_rst ;  
}ISP_WRAP_RST_CTL_S;  

typedef struct _ISP_WRAP_DMA_RST_CTL_S
{
    unsigned int y_3dnr_wr_ch_rst;     
    unsigned int uv_3dnr_wr_ch_rst;    
    unsigned int ldc_y_wr_ch_rst;      
    unsigned int ldc_uv_wr_ch_rst;     
    unsigned int wdr_raw_wr_ch_rst;    

    unsigned int main_out_y_wr_ch_rst; 
    unsigned int main_out_uv_wr_ch_rst;
    unsigned int y_3dnr_rd_ch_rst;     
    unsigned int uv_3dnr_rd_ch_rst;    
    unsigned int ldc_y_rd_ch_rst;      
    unsigned int ldc_uv_rd_ch_rst;     
    unsigned int wdr_raw_rd_ch_rst;    

    unsigned int ds0_out_y_wr_ch_rst;  
    unsigned int ds0_out_uv_wr_ch_rst; 
    unsigned int ds1_out_y_wr_ch_rst;  
    unsigned int ds1_out_uv_wr_ch_rst; 
    unsigned int ds2_out_r_wr_ch_rst;  
    unsigned int ds2_out_g_wr_ch_rst;  
    unsigned int ds2_out_b_wr_ch_rst;  

    unsigned int ds0_osd0_rd_ch_rst;   
    unsigned int ds0_osd1_rd_ch_rst;   
    unsigned int ds1_osd0_rd_ch_rst;   
    unsigned int ds1_osd1_rd_ch_rst;   
    unsigned int ds2_osd0_rd_ch_rst;   
    unsigned int ds2_osd1_rd_ch_rst;   
    unsigned int ds1_osd2_rd_ch_rst;   
    unsigned int ds2_osd2_rd_ch_rst;   
}ISP_WRAP_DMA_RST_CTL_S;

typedef struct _ISP_WRAP_PIPE_CLK_CTL_S
{
    unsigned int wrap_com_clk_en;        
    unsigned int wrap_cfg_clk_en;        
    unsigned int core_com_clk_en;        
    unsigned int core_cfg_clk_en;        
    unsigned int axi_wr_ch_en;           
    unsigned int axi_rd_ch_en;
}ISP_WRAP_PIPE_CLK_CTL_S;

typedef struct _ISP_WRAP_WDR_CLK_CTL_S
{
    unsigned int wdr_2_frame_clk_en;     
    unsigned int wdr_3_frame_clk_en; 
}ISP_WRAP_WDR_CLK_CTL_S;

typedef struct _ISP_WRAP_DS0_CLK_CTL_S
{   
    unsigned int ds0_out_clk_en;
    unsigned int ds0_out_y_ch_clk_en;    
    unsigned int ds0_out_uv_ch_clk_en;    
    unsigned int ds_out0_osd0_ch_clk_en; 
    unsigned int ds_out0_osd1_ch_clk_en;
    unsigned int ds_out0_osd2_ch_clk_en; 

} ISP_WRAP_DS0_CLK_CTL_S;

typedef struct _ISP_WRAP_DS1_CLK_CTL_S
{       
    unsigned int ds1_out_clk_en;
    unsigned int ds1_out_y_ch_clk_en;    
    unsigned int ds1_out_uv_ch_clk_en;   
    unsigned int ds_out1_osd0_ch_clk_en; 
    unsigned int ds_out1_osd1_ch_clk_en;
    unsigned int ds_out1_osd2_ch_clk_en; 
} ISP_WRAP_DS1_CLK_CTL_S;

typedef struct _ISP_WRAP_DS2_CLK_CTL_S
{         
    unsigned int ds2_out_clk_en; 
    unsigned int ds2_out_r_ch_clk_en;    
    unsigned int ds2_out_g_ch_clk_en;    
    unsigned int ds2_out_b_ch_clk_en;
    unsigned int ds_out2_osd0_ch_clk_en; 
    unsigned int ds_out2_osd1_ch_clk_en; 
} ISP_WRAP_DS2_CLK_CTL_S;

typedef struct _ISP_WRAP_WDR_MODE_CTL_S
{
    unsigned int    wdr_long_ch_mode;     
    unsigned int    wdr_long_l2_buf_en;    
    unsigned int    wdr_short_s1_buf_en; 
    unsigned int    wdr_dynamic_switch_en;
    unsigned int    wdr_long_l2_buf_depth;
}ISP_WRAP_WDR_MODE_CTL_S;

typedef struct _ISP_WRAP_CTL_MODE_S
{
    unsigned int    external_out_ch_sel;  
    unsigned int    dvp_input_ch_type; 
    unsigned int    hist_3a_out_en;          
} ISP_WRAP_CTL_MODE_S;

typedef struct _ISP_WRAP_WDR_DMA_EN_CTL_S
{
    unsigned int wdr_long_dma_en ;      
    unsigned int wdr_short_dma_en ;  
}ISP_WRAP_WDR_DMA_EN_CTL_S;

typedef struct _ISP_WRAP_DS0_DMA_EN_CTL_S
{   
    unsigned int ds_out0_dma_en ;
    unsigned int ds_out0_y_ch_dma_en ;  
    unsigned int ds_out0_uv_ch_dma_en ;
    unsigned int ds0_osd0_ch_dma_en ;   
    unsigned int ds0_osd1_ch_dma_en ;   
    unsigned int ds0_osd2_ch_dma_en ;  
} ISP_WRAP_DS0_DMA_EN_CTL_S;

typedef struct _ISP_WRAP_DS1_DMA_EN_CTL_S
{   
    unsigned int ds_out1_dma_en ;       
    unsigned int ds_out1_y_ch_dma_en ;  
    unsigned int ds_out1_uv_ch_dma_en ;   
    unsigned int ds1_osd0_ch_dma_en ;   
    unsigned int ds1_osd1_ch_dma_en ;   
    unsigned int ds1_osd2_ch_dma_en ;   
} ISP_WRAP_DS1_DMA_EN_CTL_S;

typedef struct _ISP_WRAP_DS2_DMA_EN_CTL_S
{    
    unsigned int ds_out2_dma_en ;       
    unsigned int ds_out2_r_ch_dma_en ;  
    unsigned int ds_out2_g_ch_dma_en ;  
    unsigned int ds_out2_b_ch_dma_en ;  
    unsigned int ds2_osd0_ch_dma_en ;   
    unsigned int ds2_osd1_ch_dma_en ;   
} ISP_WRAP_DS2_DMA_EN_CTL_S;

typedef struct _ISP_WRAP_3DNR_PIXEL_FORMAT_CTL_S
{
    unsigned int y_3dnr_img_format; 
    unsigned int y_3dnr_yuv_in_format;   
    unsigned int y_3dnr_img_out_format;  
    unsigned int y_3dnr_yuv422_pxl_order;
    unsigned int y_3dnr_pixel_width;  
    unsigned int uv_3dnr_img_format;   
    unsigned int uv_3dnr_yuv_in_format;  
    unsigned int uv_3dnr_img_out_format; 
    unsigned int uv_3dnr_yuv422_pxl_order;  
    unsigned int uv_3dnr_pixel_width; 
} ISP_WRAP_3DNR_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_WDR_PIXEL_FORMAT_CTL_S
{  
    unsigned int wdr_long_img_format;   
    unsigned int wdr_long_yuv_in_format;   
    unsigned int wdr_long_img_out_format;  
    unsigned int wdr_long_yuv422_pxl_order;
    unsigned int wdr_long_pixel_width;  

} ISP_WRAP_WDR_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_MAIN_PIXEL_FORMAT_CTL_S
{
    unsigned int main_out_img_format;   
    unsigned int main_out_yuv_in_format;   
    unsigned int main_out_img_out_format;  
    unsigned int main_out_yuv422_pxl_order;
    unsigned int main_out_pixel_width; 
} ISP_WRAP_MAIN_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_DS0_PIXEL_FORMAT_CTL_S
{
    unsigned int ds0_out_img_format;   
    unsigned int ds0_out_yuv_in_format;
    unsigned int ds0_out_img_out_format;   
    unsigned int ds0_out_yuv422_pxl_order; 
    unsigned int ds0_out_pixel_width; 
}ISP_WRAP_DS0_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_DS1_PIXEL_FORMAT_CTL_S
{
    unsigned int ds1_out_img_format;  
    unsigned int ds1_out_yuv_in_format;
    unsigned int ds1_out_img_out_format;   
    unsigned int ds1_out_yuv422_pxl_order; 
    unsigned int ds1_out_pixel_width;
}ISP_WRAP_DS1_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_DS2_PIXEL_FORMAT_CTL_S
{
    unsigned int ds2_out_img_format;  
    unsigned int ds2_out_yuv_in_format; 
    unsigned int ds2_out_img_out_format;   
    unsigned int ds2_out_yuv422_pxl_order; 
    unsigned int ds2_out_pixel_width;
}ISP_WRAP_DS2_PIXEL_FORMAT_CTL_S;

typedef struct _ISP_WRAP_MAIN_OUT_SIZE_S
{
    unsigned int    main_out_v_size;
    unsigned int    main_out_h_size;
} ISP_WRAP_MAIN_OUT_SIZE_S;

typedef struct _ISP_WRAP_DS0_OUT_SIZE_S
{
    unsigned int    ds0_out_v_size;
    unsigned int    ds0_out_h_size;
} ISP_WRAP_DS0_OUT_SIZE_S;

typedef struct _ISP_WRAP_DS1_OUT_SIZE_S
{
    unsigned int    ds1_out_v_size;
    unsigned int    ds1_out_h_size;
} ISP_WRAP_DS1_OUT_SIZE_S;

typedef struct _ISP_WRAP_DS2_OUT_SIZE_S
{
    unsigned int    ds2_out_v_size;
    unsigned int    ds2_out_h_size;
} ISP_WRAP_DS2_OUT_SIZE_S;

typedef struct _ISP_WRAP_OUT_SIZE_S
{
    ISP_WRAP_MAIN_OUT_SIZE_S stMainOutSize;
    ISP_WRAP_DS0_OUT_SIZE_S stDs0OutSize;
    ISP_WRAP_DS1_OUT_SIZE_S stDs1OutSize;
    ISP_WRAP_DS2_OUT_SIZE_S stDs2OutSize;
} ISP_WRAP_OUT_SIZE_S;

typedef struct _ISP_WRAP_WDR_BUF_S
{
    unsigned int    wdr_buf_base;    
    unsigned int    wdr_line_stride;
    unsigned int    wdr_buf_size;
} ISP_WRAP_WDR_BUF_S;

typedef struct _ISP_WRAP_3DNR_BUF_S
{
    unsigned int    y_3dnr_buf_base;
    unsigned int    uv_3dnr_buf_base;
        
    unsigned int    y_3dnr_line_stride;
    unsigned int    uv_3dnr_line_stride;
    unsigned int      nr3d_buf_size;   
} ISP_WRAP_3DNR_BUF_S;

typedef struct _ISP_WRAP_LDC_BUF_S
{
    unsigned int    ldc_y_buf_base;    
    unsigned int    ldc_uv_buf_base;    
    unsigned int  ldc_y_line_stride; 
    unsigned int  ldc_uv_line_stride;
    unsigned int    ldc_buf_size;    
} ISP_WRAP_LDC_BUF_S;

typedef struct _ISP_WRAP_MAIN_BUF_S
{

    unsigned int    main_y_buf0_base;   
    unsigned int    main_y_buf1_base;   
    unsigned int    main_uv_buf0_base;  
    unsigned int    main_uv_buf1_base; 
    unsigned int    main_y_line_stride; 
    unsigned int    main_uv_line_stride;
    unsigned int    main_buf_size;
} ISP_WRAP_MAIN_BUF_S;


typedef struct _ISP_WRAP_DS0_BUF_S
{

    unsigned int    ds0_y_buf0_base;   
    unsigned int    ds0_y_buf1_base;   
    unsigned int    ds0_uv_buf0_base;  
    unsigned int    ds0_uv_buf1_base;  
    unsigned int    ds0_y_line_stride; 
    unsigned int    ds0_uv_line_stride;
    unsigned int    ds0_buf_size;    

} ISP_WRAP_DS0_BUF_S;

typedef struct _ISP_WRAP_DS1_BUF_S
{

    unsigned int    ds1_y_buf0_base;   
    unsigned int    ds1_y_buf1_base;   
    unsigned int    ds1_uv_buf0_base;  
    unsigned int    ds1_uv_buf1_base;  
    unsigned int    ds1_y_line_stride; 
    unsigned int    ds1_uv_line_stride;
    unsigned int    ds1_buf_size; 
} ISP_WRAP_DS1_BUF_S;

typedef struct _ISP_WRAP_DS2_BUF_S
{
    unsigned int    ds2_r_buf0_base;   
    unsigned int    ds2_r_buf1_base;   
    unsigned int    ds2_g_buf0_base;   
    unsigned int    ds2_g_buf1_base;   
    unsigned int    ds2_b_buf0_base;   
    unsigned int    ds2_b_buf1_base;   
    unsigned int    ds2_r_line_stride; 
    unsigned int    ds2_g_line_stride;
    unsigned int    ds2_b_line_stride;
    unsigned int    ds2_buf_size;     
} ISP_WRAP_DS2_BUF_S;

typedef struct _ISP_WRAP_DMA_MODE_S
{
    unsigned int    wr_dma_arb_mode;
    unsigned int    rd_dma_arb_mode;    
    unsigned int    wr_dma_ch_weight[ISP_DMA_CH_NUM];
    unsigned int    rd_dma_ch_weight[ISP_DMA_CH_NUM];
    unsigned int    wr_dma_ch_priority[ISP_DMA_CH_NUM];
    unsigned int    rd_dma_ch_priority[ISP_DMA_CH_NUM];
    unsigned int    wr_dma_ch_id[ISP_DMA_CH_NUM];
    unsigned int    rd_dma_ch_id[ISP_DMA_CH_NUM];
} ISP_WRAP_DMA_MODE_S;

typedef struct _ISP_WRAP_DMA_CH_CFG_S
{
    unsigned int    wr_ch_outst; 
    unsigned int    rd_ch_outst; 
    unsigned int    wr_ch_burstl;
    unsigned int    rd_ch_burstl;
        
    unsigned int    y_uv_noncon_en;  
    unsigned int    y_uv_swap_en;    
    unsigned int    wr_int_line_num;
    
    unsigned int    wr_err_dec_en;
    unsigned int    rd_err_dec_en;

    unsigned int    wr_outst_en;
    unsigned int    rd_outst_en;
} ISP_WRAP_DMA_CFG_CH_S;

typedef struct _ISP_WRAP_DMA_CFG_S
{
    ISP_WRAP_DMA_CFG_CH_S stDmaChcfg[ISP_DMA_CH_NUM];
} ISP_WRAP_DMA_CFG_S;

typedef struct _ISP_WRAP_DMA_CH_ERR_DEC_CFG_S
{
    unsigned int    wr_err_dec_unit; 
    unsigned int    rd_err_dec_unit;
    unsigned int    wr_err_gen_thr; 
    unsigned int    rd_err_gen_thr; 
} ISP_WRAP_DMA_CH_ERR_DEC_CFG_S;

typedef struct _ISP_WRAP_DMA_ERR_DEC_CFG_S
{
    ISP_WRAP_DMA_CH_ERR_DEC_CFG_S stDmaChErrDecCfg[ISP_DMA_CH_NUM];
} ISP_WRAP_DMA_ERR_DEC_CFG_S;

typedef struct _ISP_WRAP_DMA_CH_ERR_STATUS_S
{
    unsigned int    err_y_wr_status;    
    unsigned int    err_uv_wr_status;  
    unsigned int    err_raw_read_status;
    unsigned int    err_wr_rst_req;
    unsigned int    err_rd_rst_req;     
    unsigned int    err_wr_ch_state;    
    unsigned int    err_rd_ch_state;
    unsigned int    reserved0;    
} ISP_WRAP_DMA_CH_ERR_STATUS_S;

typedef struct _ISP_WRAP_DMA_ERR_STATUS_S
{
    ISP_WRAP_DMA_CH_ERR_STATUS_S stDmaErrStatus[ISP_DMA_CH_NUM];
} ISP_WRAP_DMA_ERR_STATUS_S;

typedef struct _ISP_WRAP_DMA_ATTR_S
{
    ISP_WRAP_DMA_MODE_S stDmaChMode;
    ISP_WRAP_DMA_CFG_S stDmaCfg;
    ISP_WRAP_DMA_ERR_DEC_CFG_S stErrDecCfg;
    ISP_WRAP_DMA_ERR_STATUS_S stErrStatus;
} ISP_WRAP_DMA_ATTR_S;

typedef struct _ISP_WRAP_AXI_CTL_S
{
    unsigned int    axi_wr_ch_rst_req;
    unsigned int    axi_rd_ch_rst_req;
    unsigned int    axi_wr_ch_state;  
    unsigned int    axi_rd_ch_state;  
} ISP_WRAP_AXI_CTL_S;

typedef struct _ISP_WRAP_CONFIG_DONE_S
{
    unsigned int wrap_config_done_en;         
    unsigned int sen_reg_pro_en;      
    unsigned int int_polarity;    
} ISP_WRAP_CONFIG_DONE_S;

typedef struct _ISP_WRAP_CORE_INT_CTL_S
{
    unsigned int int_raw_in;       
    unsigned int int_3a;           
    unsigned int raw_int;          
    unsigned int rgb_int;          
    unsigned int yuv_int;          
    unsigned int ldc_int;          
    unsigned int main_out_int;     
    unsigned int isp_awb_int;      
    unsigned int isp_ae_int;       
    unsigned int isp_af_int;       
    
    unsigned int int_raw_in_mask;  
    unsigned int int_3a_mask;      
    unsigned int raw_int_mask;     
    unsigned int rgb_int_mask;     
    unsigned int yuv_int_mask;     
    unsigned int ldc_int_mask;     
    unsigned int main_out_int_mask;
    unsigned int isp_awb_int_mask; 
    unsigned int isp_ae_int_mask;  
    unsigned int isp_af_int_mask;  

} ISP_WRAP_CORE_INT_CTL_S;

typedef struct _ISP_WRAP_DMA_WR_INT_MASK0_S
{
    unsigned int wr_3dnr_y_frm_end_int_mask;           
    unsigned int wr_3dnr_y_line_base_int_mask;         
    unsigned int wr_3dnr_y_err_frm_end_int_mask;       
    unsigned int wr_3dnr_y_err_immediate_int_mask;     
    unsigned int wr_3dnr_uv_frm_end_int_mask;          
    unsigned int wr_3dnr_uv_line_base_int_mask;        
    unsigned int wr_3dnr_uv_err_frm_end_int_mask;      
    unsigned int wr_3dnr_uv_err_immediate_int_mask;    
    unsigned int ldc_wr_y_frm_end_int_mask;            
    unsigned int ldc_wr_y_line_base_int_mask;          
    unsigned int ldc_wr_y_err_frm_end_int_mask;        
    unsigned int ldc_wr_y_err_immediate_int_mask;      
    unsigned int ldc_wr_uv_frm_end_int_mask;           
    unsigned int ldc_wr_uv_line_base_int_mask;         
    unsigned int ldc_wr_uv_err_frm_end_int_mask;       
    unsigned int ldc_wr_uv_err_immediate_int_mask;     
    unsigned int wdr_wr_raw_frm_end_int_mask;          
    unsigned int wdr_wr_raw_line_base_int_mask;        
    unsigned int wdr_wr_raw_err_frm_end_int_mask;      
    unsigned int wdr_wr_raw_err_immediate_int_mask;    
                 
    unsigned int main_out_wr_y_frm_end_int_mask;       
    unsigned int main_out_wr_y_line_base_int_mask;     
    unsigned int main_out_wr_y_err_frm_end_int_mask;   
    unsigned int main_out_wr_y_err_immediate_int_mask; 
    unsigned int main_out_wr_uv_frm_end_int_mask;      
    unsigned int main_out_wr_uv_line_base_int_mask;    
    unsigned int main_out_wr_uv_err_frm_end_int_mask;  
    unsigned int main_out_wr_uv_err_immediate_int_mask;

} ISP_WRAP_DMA_WR_INT_MASK0_S;

typedef struct _ISP_WRAP_DMA_WR_INT_MASK1_S
{
    unsigned int ds0_out_wr_y_frm_end_mask;        
    unsigned int ds0_out_wr_y_line_base_mask;      
    unsigned int ds0_out_wr_y_err_frm_end_mask;    
    unsigned int ds0_out_wr_y_err_immediate_mask;  
    unsigned int ds0_out_wr_uv_frm_end_mask;       
    unsigned int ds0_out_wr_uv_line_base_mask;     
    unsigned int ds0_out_wr_uv_err_frm_end_mask;   
    unsigned int ds0_out_wr_uv_err_immediate_mask; 
    unsigned int ds1_out_wr_y_frm_end_mask;        
    unsigned int ds1_out_wr_y_line_base_mask;      
    unsigned int ds1_out_wr_y_err_frm_end_mask;    
    unsigned int ds1_out_wr_y_err_immediate_mask;  
    unsigned int ds1_out_wr_uv_frm_end_mask;       
    unsigned int ds1_out_wr_uv_line_base_mask;     
    unsigned int ds1_out_wr_uv_err_frm_end_mask;   
    unsigned int ds1_out_wr_uv_err_immediate_mask; 
    unsigned int ds2_out_wr_r_frm_end_mask;        
    unsigned int ds2_out_wr_r_line_base_mask;      
    unsigned int ds2_out_wr_r_err_frm_end_mask;    
    unsigned int ds2_out_wr_r_err_immediate_mask;  
    unsigned int ds2_out_wr_g_frm_end_mask;        
    unsigned int ds2_out_wr_g_line_base_mask;      
    unsigned int ds2_out_wr_g_err_frm_end_mask;    
    unsigned int ds2_out_wr_g_err_immediate_mask;  
    unsigned int ds2_out_wr_b_frm_end_mask;        
    unsigned int ds2_out_wr_b_line_base_mask;      
    unsigned int ds2_out_wr_b_err_frm_end_mask;    
    unsigned int ds2_out_wr_b_err_immediate_mask;  
} ISP_WRAP_DMA_WR_INT_MASK1_S;

typedef struct _ISP_WRAP_DMA_RD_INT_MASK0_S
{
    unsigned int rd_3dnr_y_frm_end_int_mask;       
    unsigned int rd_3dnr_y_line_base_int_mask;     
    unsigned int rd_3dnr_y_err_frm_end_int_mask;   
    unsigned int rd_3dnr_y_err_immediate_int_mask; 
    unsigned int rd_3dnr_uv_frm_end_int_mask;      
    unsigned int rd_3dnr_uv_line_base_int_mask;    
    unsigned int rd_3dnr_uv_err_frm_end_int_mask;  
    unsigned int rd_3dnr_uv_err_immediate_int_mask;
    unsigned int ldc_rd_y_frm_end_int_mask;        
    unsigned int ldc_rd_y_line_base_int_mask;      
    unsigned int ldc_rd_y_err_frm_end_int_mask;    
    unsigned int ldc_rd_y_err_immediate_int_mask;  
    unsigned int ldc_rd_uv_frm_end_int_mask;       
    unsigned int ldc_rd_uv_line_base_int_mask;     
    unsigned int ldc_rd_uv_err_frm_end_int_mask;   
    unsigned int ldc_rd_uv_err_immediate_int_mask; 
    unsigned int wdr_rd_raw_frm_end_int_mask;      
    unsigned int wdr_rd_raw_line_base_int_mask;    
    unsigned int wdr_rd_raw_err_frm_end_int_mask;  
    unsigned int wdr_rd_raw_err_immediate_int_mask;              

} ISP_WRAP_DMA_RD_INT_MASK0_S;

typedef struct _ISP_WRAP_INT_CTL_S
{
    ISP_WRAP_CORE_INT_CTL_S stCoreIntCtl;
    ISP_WRAP_DMA_WR_INT_MASK0_S stDmaWRMask0;
    ISP_WRAP_DMA_WR_INT_MASK1_S stDmaWRMask1;
    ISP_WRAP_DMA_RD_INT_MASK0_S stDmaRDMask0;
}ISP_WRAP_INT_CTL_S;

typedef struct _ISP_VERSION_S
{
    unsigned int    isp_version_minor;    
    unsigned int    isp_version_major;    
    unsigned int    isp_license_version;  
    unsigned int    isp_wdr_l2_info;      
    
    unsigned int    isp_wdr_s1_info;      
    unsigned int    isp_main_output_info; 
    unsigned int    isp_ds0_output_info;  
    unsigned int    isp_ds1_output_info;  
    
    unsigned int    isp_ds2_output_info;  
         
} ISP_VERSION_S;

typedef struct _ISP_RELEASE_INFO_S
{
    unsigned int    isp_release_day;      
    unsigned int    isp_release_month;    
    unsigned int    isp_release_year_low; 
    unsigned int    isp_release_year_high;
} ISP_RELEASE_INFO_S;

typedef struct _ISP_WRAP_WDR_ATTR_S
{ 
    ISP_WRAP_WDR_CLK_CTL_S          stWdrClkCtl;
    ISP_WRAP_WDR_MODE_CTL_S         stWdrModeCtl;
    ISP_WRAP_WDR_DMA_EN_CTL_S       stWdrDmaEn;
    ISP_WRAP_WDR_PIXEL_FORMAT_CTL_S stWdrPixelFormat;
    ISP_WRAP_WDR_BUF_S              stWdrBuf;
}ISP_WRAP_WDR_ATTR_S;

typedef struct _ISP_WRAP_3DNR_ATTR_S
{ 
    unsigned int                    clk_3dnr_en; 
    unsigned int                    nr3d_fbcd_en;
    unsigned int                    nr3d_mv_out_en;
    unsigned int                    pipe_3dnr_dma_en ; 
    ISP_WRAP_3DNR_PIXEL_FORMAT_CTL_S st3DnrPixelFormat;
    ISP_WRAP_3DNR_BUF_S             st3dnrBuf;
}ISP_WRAP_3DNR_ATTR_S;

typedef struct _ISP_WRAP_LDC_ATTR_S
{ 
    unsigned int                    ldc_clk_en;
    unsigned int                    ldc_dma_en ;
    ISP_WRAP_LDC_BUF_S              stLdcBuf;
}ISP_WRAP_LDC_ATTR_S;

typedef struct _ISP_WRAP_MAINOUT_ATTR_S
{ 
    unsigned int                    main_out_clk_en;
    unsigned int                    main_out_dma_en;
    unsigned int                    pix_remap_main_en;
    ISP_WRAP_MAIN_PIXEL_FORMAT_CTL_S stMainPixelFormat;
    ISP_WRAP_MAIN_OUT_SIZE_S        stMainOutSize;
    ISP_WRAP_MAIN_BUF_S             stMainBuf;
}ISP_WRAP_MAINOUT_ATTR_S;

typedef struct _ISP_WRAP_DS0OUT_ATTR_S
{ 
    ISP_WRAP_DS0_CLK_CTL_S          stDs0ClkCtl;
    unsigned int                    pix_remap_out0_en;
    ISP_WRAP_DS0_DMA_EN_CTL_S       stDs0DmaEn;
    ISP_WRAP_DS0_PIXEL_FORMAT_CTL_S stDs0PixelFormat;
    ISP_WRAP_DS0_OUT_SIZE_S         stDs0OutSize;
    ISP_WRAP_DS0_BUF_S              stDs0Buf;
}ISP_WRAP_DS0OUT_ATTR_S;

typedef struct _ISP_WRAP_DS1OUT_ATTR_S
{ 
    ISP_WRAP_DS1_CLK_CTL_S          stDs1ClkCtl;
    unsigned int                   pix_remap_out1_en;
    ISP_WRAP_DS1_DMA_EN_CTL_S       stDs1DmaEn;
    ISP_WRAP_DS1_PIXEL_FORMAT_CTL_S stDs1PixelFormat;
    ISP_WRAP_DS1_OUT_SIZE_S         stDs1OutSize;
    ISP_WRAP_DS1_BUF_S              stDs1Buf;
}ISP_WRAP_DS1OUT_ATTR_S;

typedef struct _ISP_WRAP_DS2OUT_ATTR_S
{ 
    ISP_WRAP_DS2_CLK_CTL_S          stDs2ClkCtl;
    ISP_WRAP_DS2_DMA_EN_CTL_S       stDs2DmaEn;
    ISP_WRAP_DS2_PIXEL_FORMAT_CTL_S stDs2PixelFormat;
    ISP_WRAP_DS2_OUT_SIZE_S         stDs2OutSize;
    ISP_WRAP_DS2_BUF_S              stDs2Buf;
}ISP_WRAP_DS2OUT_ATTR_S;

/*
*F2K   
*/
void Isp_Drv_F2k_Wrap_SetPipeReset(struct k510_isp_device *isp,ISP_WRAP_RST_CTL_S *pstRstCtl);
void Isp_Drv_F2k_Wrap_SetDmaReset(struct k510_isp_device *isp,ISP_WRAP_DMA_RST_CTL_S *pstDmaRstCtl);
int Isp_Drv_F2k_Wrap_SetModeCtl_OutSel(struct k510_isp_device *isp,unsigned int external_out_ch_sel);
int Isp_Drv_F2k_Wrap_SetModeCtl_DvpMode(struct k510_isp_device *isp,unsigned int dvp_input_ch_type);
int Isp_Drv_F2k_Wrap_SetModeCtl_3dnr_mfbcd_en(struct k510_isp_device *isp,unsigned int nr3d_mfbcd_en);
int Isp_Drv_F2k_Wrap_SetModeCtl_3dnr_mv_en(struct k510_isp_device *isp,unsigned int nr3d_mv_out_en);
int Isp_Drv_F2k_Wrap_SetModeCtl_3a_out_en(struct k510_isp_device *isp,unsigned int hist_3a_out_en);
int Isp_Drv_F2k_Wrap_SetModeCtl_remap_main_en(struct k510_isp_device *isp,unsigned int pix_remap_main_en);
int Isp_Drv_F2k_Wrap_SetModeCtl_remap_out0_en(struct k510_isp_device *isp,unsigned int pix_remap_out0_en);
int Isp_Drv_F2k_Wrap_SetModeCtl_remap_out1_en(struct k510_isp_device *isp,unsigned int pix_remap_out1_en);
void Isp_Drv_F2k_Wrap_SetComClkCtl(struct k510_isp_device *isp,ISP_WRAP_PIPE_CLK_CTL_S *pstPipeClkCtl);
int Isp_Drv_F2k_Wrap_SetDma(struct k510_isp_device *isp,ISP_WRAP_DMA_ATTR_S *pstDmaAttr);
int Isp_Drv_F2k_Wrap_SetwdrBuf(struct k510_isp_device *isp,ISP_WRAP_WDR_BUF_S *pstWdrBuf);
int Isp_Drv_F2k_Wrap_Set3dnrBuf(struct k510_isp_device *isp,ISP_WRAP_3DNR_BUF_S *pst3dnrBuf);
int Isp_Drv_F2k_Wrap_SetLdcBuf(struct k510_isp_device *isp,ISP_WRAP_LDC_BUF_S *pstLdcBuf);
int Isp_Drv_F2k_Wrap_SetMainBuf(struct k510_isp_device *isp,ISP_WRAP_MAIN_BUF_S *pstMainBuf);
int Isp_Drv_F2k_Wrap_SetDs0Buf(struct k510_isp_device *isp,ISP_WRAP_DS0_BUF_S *pstDs0Buf);
int Isp_Drv_F2k_Wrap_SetDs1Buf(struct k510_isp_device *isp,ISP_WRAP_DS1_BUF_S *pstDs1Buf);
int Isp_Drv_F2k_Wrap_SetDs2Buf(struct k510_isp_device *isp,ISP_WRAP_DS2_BUF_S *pstDs2Buf);
int Isp_Drv_F2k_Wrap_SetAxiCtl(struct k510_isp_device *isp,ISP_WRAP_AXI_CTL_S *pstAxiCtl);
int Isp_Drv_F2k_Wrap_SetConfigdoneCtl(struct k510_isp_device *isp,ISP_WRAP_CONFIG_DONE_S *pstConfigDone);
int Isp_Drv_F2k_Wrap_SetCoreIntCtlMask(struct k510_isp_device *isp,ISP_WRAP_CORE_INT_CTL_S *pstCoreIntCtl);
int Isp_Drv_F2k_Wrap_SetDmaWRIntMask0(struct k510_isp_device *isp,ISP_WRAP_DMA_WR_INT_MASK0_S *pstDmaWRIntMask0);
int Isp_Drv_F2k_Wrap_SetDmaWRIntMask1(struct k510_isp_device *isp,ISP_WRAP_DMA_WR_INT_MASK1_S *pstDmaWRIntMask1);
int Isp_Drv_F2k_Wrap_SetDmaRDIntMask0(struct k510_isp_device *isp,ISP_WRAP_DMA_RD_INT_MASK0_S *pstDmaRDIntMask0);
int Isp_Drv_F2k_Wrap_SetWdr(struct k510_isp_device *isp,ISP_WRAP_WDR_ATTR_S *pstWdrAttr);
int Isp_Drv_F2k_Wrap_Set3Dnr(struct k510_isp_device *isp,ISP_WRAP_3DNR_ATTR_S *pst3DnrAttr);
int Isp_Drv_F2k_Wrap_SetLdc(struct k510_isp_device *isp,ISP_WRAP_LDC_ATTR_S *pstLdcAttr);
int Isp_Drv_F2k_Wrap_SetMainOut(struct k510_isp_device *isp,ISP_WRAP_MAINOUT_ATTR_S *pstMainOutAttr);
int Isp_Drv_F2k_Wrap_SetDs0Out(struct k510_isp_device *isp,ISP_WRAP_DS0OUT_ATTR_S *pstDs0OutAttr);
int Isp_Drv_F2k_Wrap_SetDs1Out(struct k510_isp_device *isp,ISP_WRAP_DS1OUT_ATTR_S *pstDs1OutAttr);
int Isp_Drv_F2k_Wrap_SetDs2Out(struct k510_isp_device *isp,ISP_WRAP_DS2OUT_ATTR_S *pstDs2OutAttr);
unsigned int Isp_Drv_F2k_Wrap_GetRevison(struct k510_isp_device *isp);
/*
*R2K                      
*/
void Isp_Drv_R2k_Wrap_SetPipeReset(struct k510_isp_device *isp,ISP_WRAP_RST_CTL_S *pstRstCtl);
void Isp_Drv_R2k_Wrap_SetDmaReset(struct k510_isp_device *isp,ISP_WRAP_DMA_RST_CTL_S *pstDmaRstCtl);
void Isp_Drv_R2k_Wrap_SetReset(struct k510_isp_device *isp);
int Isp_Drv_R2k_Wrap_SetModeCtl_OutSel(struct k510_isp_device *isp,unsigned int external_out_ch_sel);
int Isp_Drv_R2k_Wrap_SetModeCtl_DvpMode(struct k510_isp_device *isp,unsigned int dvp_input_ch_type);
int Isp_Drv_R2k_Wrap_SetModeCtl_3a_out_en(struct k510_isp_device *isp,unsigned int hist_3a_out_en);
int Isp_Drv_R2k_Wrap_SetModeCtl_remap_main_en(struct k510_isp_device *isp,unsigned int pix_remap_main_en);
int Isp_Drv_R2k_Wrap_SetModeCtl_remap_out0_en(struct k510_isp_device *isp,unsigned int pix_remap_out0_en);
int Isp_Drv_R2k_Wrap_SetModeCtl_remap_out1_en(struct k510_isp_device *isp,unsigned int pix_remap_out1_en);
void Isp_Drv_R2k_Wrap_SetComClkCtl(struct k510_isp_device *isp,ISP_WRAP_PIPE_CLK_CTL_S *pstPipeClkCtl);
int Isp_Drv_R2k_Wrap_SetDma(struct k510_isp_device *isp,ISP_WRAP_DMA_ATTR_S *pstDmaAttr);
int Isp_Drv_R2k_Wrap_SetLdcBuf(struct k510_isp_device *isp,ISP_WRAP_LDC_BUF_S *pstLdcBuf);
int Isp_Drv_R2k_Wrap_SetMainBuf(struct k510_isp_device *isp,ISP_WRAP_MAIN_BUF_S *pstMainBuf);
int Isp_Drv_R2k_Wrap_SetDs0Buf(struct k510_isp_device *isp,ISP_WRAP_DS0_BUF_S *pstDs0Buf);
int Isp_Drv_R2k_Wrap_SetDs1Buf(struct k510_isp_device *isp,ISP_WRAP_DS1_BUF_S *pstDs1Buf);
int Isp_Drv_R2k_Wrap_SetDs2Buf(struct k510_isp_device *isp,ISP_WRAP_DS2_BUF_S *pstDs2Buf);
int Isp_Drv_R2k_Wrap_SetAxiCtl(struct k510_isp_device *isp,ISP_WRAP_AXI_CTL_S *pstAxiCtl);
int Isp_Drv_R2k_Wrap_SetConfigdoneCtl(struct k510_isp_device *isp,ISP_WRAP_CONFIG_DONE_S *pstConfigDone);
int Isp_Drv_R2k_Wrap_SetCoreIntCtlMask(struct k510_isp_device *isp,ISP_WRAP_CORE_INT_CTL_S *pstCoreIntCtl);
int Isp_Drv_R2k_Wrap_SetDmaWRIntMask0(struct k510_isp_device *isp,ISP_WRAP_DMA_WR_INT_MASK0_S *pstDmaWRIntMask0);
int Isp_Drv_R2k_Wrap_SetDmaWRIntMask1(struct k510_isp_device *isp,ISP_WRAP_DMA_WR_INT_MASK1_S *pstDmaWRIntMask1);
int Isp_Drv_R2k_Wrap_SetDmaRDIntMask0(struct k510_isp_device *isp,ISP_WRAP_DMA_RD_INT_MASK0_S *pstDmaRDIntMask0);
int Isp_Drv_R2k_Wrap_SetLdc(struct k510_isp_device *isp,ISP_WRAP_LDC_ATTR_S *pstLdcAttr);
int Isp_Drv_R2k_Wrap_SetMainOut(struct k510_isp_device *isp,ISP_WRAP_MAINOUT_ATTR_S *pstMainOutAttr);
int Isp_Drv_R2k_Wrap_SetDs0Out(struct k510_isp_device *isp,ISP_WRAP_DS0OUT_ATTR_S *pstDs0OutAttr);
int Isp_Drv_R2k_Wrap_SetDs1Out(struct k510_isp_device *isp,ISP_WRAP_DS1OUT_ATTR_S *pstDs1OutAttr);
int Isp_Drv_R2k_Wrap_SetDs2Out(struct k510_isp_device *isp,ISP_WRAP_DS2OUT_ATTR_S *pstDs2OutAttr);
//
unsigned int Isp_Drv_R2k_Wrap_GetRevison(struct k510_isp_device *isp);
#endif /*_ISP_WRAP_DRV_H_*/
