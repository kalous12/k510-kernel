/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _VI_PIPE_REG_H_
#define _VI_PIPE_REG_H_

union U_VI_PIPE_CTRL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    reserved0                      : 10  ; /* [9 ..0 ]  */
        unsigned int    win_mode_en                    : 1   ; /* [10]      */
        unsigned int    emb_en                         : 1   ; /* [11]      */
        unsigned int    hsync_vsync_out_en             : 1   ; /* [12]      */
        unsigned int    sensor_input_swap_en           : 1   ; /* [13]      */
        unsigned int    reserved1                      : 2   ; /* [15..14]  */
        unsigned int    sen_mipi_clk_pol               : 1   ; /* [16]      */
        unsigned int    sen_mipi_vsync_pol             : 1   ; /* [17]      */
        unsigned int    sen_mipi_hsync_pol             : 1   ; /* [18]      */
        unsigned int    sen_mipi_field_pol             : 1   ; /* [19]      */
        unsigned int    isp_clk_pol                    : 1   ; /* [20]      */
        unsigned int    isp_vsync_pol                  : 1   ; /* [21]      */
        unsigned int    isp_hsync_pol                  : 1   ; /* [22]      */
        unsigned int    isp_field_pol                  : 1   ; /* [23]      */
        unsigned int    sync_source_sel                : 1   ; /* [24]      */
        unsigned int    input_ch_sel                   : 1   ; /* [25]      */
        unsigned int    ch_mode_sel                    : 1   ; /* [26]      */
        unsigned int    emb_enable                     : 1   ; /* [27]      */
        unsigned int    emb_mode_sel                   : 1   ; /* [28]      */
        unsigned int    emb_ecc_mode_en                : 1   ; /* [29]      */
        unsigned int    sync_code_remove_en            : 1   ; /* [30]      */
        unsigned int    sync_code_ext_en               : 1   ; /* [31]      */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
};

union U_VI_PIPE_TIMING_CTRL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    input_pixel_type               : 1   ; /* [0 ]      */
        unsigned int    input_yuv_format               : 1   ; /* [1 ]      */
        unsigned int    output_yuv_format              : 1   ; /* [2 ]      */
        unsigned int    yuv_order                      : 2   ; /* [4 ..3 ]  */
        unsigned int    raw_pixel_width                : 3   ; /* [7 ..5 ]  */
        unsigned int    data_out_timming_ctrl          : 4   ; /* [11..8 ]  */
        unsigned int    sync_mode                      : 3   ; /* [14..12]  */
        unsigned int    sync_pulse_mode                : 1   ; /* [15]      */
        unsigned int    emb_timeout_set                : 12  ; /* [27..16]  */
        unsigned int    reserved0                      : 4   ; /* [31..28]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
};

union U_VI_PIPE_SYNC_WIDTH
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_vsync_width              : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    image_hsync_width              : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
};

union U_VI_PIPE_IMAGE_SIZE_TOTAL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_v_size_total             : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    image_h_size_total             : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_IMAGE_SIZE_ACTIVE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_v_size_active            : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    image_h_size_active            : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_IMAGE_START
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_v_start                  : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    image_h_start                  : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_FRM_BASE_ADDR0_Y
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_frm_y_base_addr0         : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_FRM_BASE_ADDR1_Y
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_frm_y_base_addr1         : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_FRM_BASE_ADDR0_UV
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_frm_uv_base_addr0        : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_FRM_BASE_ADDR1_UV
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_frm_uv_base_addr1        : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_LINE_STRIDE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    image_y_line_stride            : 13  ; /* [12..0 ]  */
        unsigned int    reserved0                      : 3   ; /* [15..13]  */
        unsigned int    image_uv_line_stride           : 13  ; /* [28..16]  */
        unsigned int    reserved1                      : 3   ; /* [31..29]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_DMA_CTL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    dma_y_module_en                : 1   ; /* [0 ]      */
        unsigned int    dma_uv_module_en               : 1   ; /* [1 ]      */
        unsigned int    dma_tpg_read_en                : 1   ; /* [2 ]      */
        unsigned int    reserved0                      : 1   ; /* [3 ]      */
        unsigned int    sync_out_en                    : 1   ; /* [4 ]      */
        unsigned int    sw_triger_en                   : 1   ; /* [5 ]      */
        unsigned int    dma_wr_ch_err_dec_en           : 1   ; /* [6 ]      */
        unsigned int    dma_rd_ch_err_dec_en           : 1   ; /* [7 ]      */
        unsigned int    reserved1                      : 24  ; /* [31..8 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_CTL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_en                         : 1   ; /* [0 ]      */
        unsigned int    tpg_frame_ratio_mode           : 1   ; /* [1 ]      */
        unsigned int    tpg_sync_timing_gen_mode       : 1   ; /* [2 ]      */
        unsigned int    reserved0                      : 5   ; /* [7 ..3 ]  */
        unsigned int    tpg_frame_ratio_fast_slow      : 8   ; /* [15..8 ]  */
        unsigned int    reserved1                      : 16  ; /* [31..16]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_TOF
{
    /* Define the struct bits */
    struct
    {
        unsigned int    vi_pipe_tpg_tof_frm_stride     : 20  ; /* [19..0 ]  */
        unsigned int    vi_pipe_tpg_tof_frm_num        :  4  ; /* [23..20]  */
        unsigned int    reserved0                      : 7   ; /* [30..24]  */
        unsigned int    tof_mode_enable                : 1   ; /* [31]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;


union U_VI_PIPE_TPG_SIZE_TOTAL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_vsize_total                : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    tpg_hsize_total                : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_SIZE_ACTIVE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_vsize_active               : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    tpg_hsize_active               : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_START
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_v_start                    : 14  ; /* [13..0 ]  */
        unsigned int    reserved0                      : 2   ; /* [15..14]  */
        unsigned int    tpg_h_start                    : 14  ; /* [29..16]  */
        unsigned int    reserved1                      : 2   ; /* [31..30]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_FRM_BASE_ADDR0_Y
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_y_frm_base_addr0           : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_TPG_FRM_BASE_ADDR1_Y
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_y_frm_base_addr1           : 32  ; /* [31..0 ]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;
 
union U_VI_PIPE_TPG_LINE_STRIDE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    tpg_y_line_stride              : 13  ; /* [12..0 ]  */
        unsigned int    reserved0                      : 19  ; /* [31..13]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_DMA_MODE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    dma_y_outstand                 : 4   ; /* [3 ..0 ]  */
        unsigned int    dma_uv_outstand                : 4   ; /* [7 ..4 ]  */
        unsigned int    dma_raw_outstand               : 4   ; /* [11..8 ]  */
        unsigned int    dma_y_blen                     : 1   ; /* [12]      */
        unsigned int    dma_uv_blen                    : 1   ; /* [13]      */
        unsigned int    dma_raw_blen                   : 1   ; /* [14]      */
        unsigned int    dma_y_uv_out_swap              : 1   ; /* [15]      */
        unsigned int    dma_int_line_num               : 12  ; /* [27..16]  */
        unsigned int    reserved0                      : 4   ; /* [31..28]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_DMA_ERROR_MODE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    dma_wr_err_dec_unit            : 8   ; /* [7 ..0 ]  */
        unsigned int    dma_rd_err_dec_unit            : 8   ; /* [15..8 ]  */
        unsigned int    dma_wr_err_int_gen_th          : 8   ; /* [23..16]  */
        unsigned int    reserved0                      : 5   ; /* [28..24]  */
        unsigned int    dma_rd_err_int_gen_th          : 3   ; /* [31..29]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_DMA_RST_REQ
{
    /* Define the struct bits */
    struct
    {
        unsigned int    dma_y_wr_frm_cnt_clr           : 1   ; /* [0 ]      */
        unsigned int    dma_y_wr_fifo_err_cnt_clr      : 1   ; /* [1 ]      */
        unsigned int    dma_y_wr_bw_max_clr            : 1   ; /* [2 ]      */
        unsigned int    dma_y_wr_bw_total_clr          : 1   ; /* [3 ]      */
        unsigned int    reserved0                      : 4   ; /* [7 ..4 ]  */
        unsigned int    dma_uv_wr_frm_cnt_clr          : 1   ; /* [8 ]      */
        unsigned int    dma_uv_wr_fifo_err_cnt_clr     : 1   ; /* [9 ]      */
        unsigned int    dma_uv_wr_bw_max_clr           : 1   ; /* [10]      */
        unsigned int    dma_uv_wr_bw_total_clr         : 1   ; /* [11]      */
        unsigned int    reserved1                      : 4   ; /* [15..12]  */
        unsigned int    dma_raw_rd_frm_cnt_clr         : 1   ; /* [16]      */
        unsigned int    dma_raw_rd_fifo_err_cnt_clr    : 1   ; /* [17]      */
        unsigned int    dma_raw_rd_bw_max_clr          : 1   ; /* [18]      */
        unsigned int    dma_raw_rd_bw_total_clr        : 1   ; /* [19]      */
        unsigned int    reserved2                      : 4   ; /* [23..20]  */
        unsigned int    dma_y_wr_rst_req               : 1   ; /* [24]      */
        unsigned int    dma_uv_wr_rst_req              : 1   ; /* [25]      */
        unsigned int    dma_raw_rd_rst_req             : 1   ; /* [26]      */
        unsigned int    reserved3                      : 1   ; /* [27]      */
        unsigned int    dma_y_wr_idle                  : 1   ; /* [28]      */
        unsigned int    dma_uv_wr_idle                 : 1   ; /* [29]      */
        unsigned int    dma_raw_rd_idle                : 1   ; /* [30]      */
        unsigned int    reserved4                      : 1   ; /* [31]      */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_CONFIG_DONE
{
    /* Define the struct bits */
    struct
    {
        unsigned int    pipe_config_done               : 1   ; /* [0 ]      */
        unsigned int    reserved0                      : 15  ; /* [15..1 ]  */
        unsigned int    sens_reg_wr_protect            : 1   ; /* [16]      */
        unsigned int    int_pol                        : 1   ; /* [17]      */
        unsigned int    reserved1                      : 14  ; /* [31..18]  */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

union U_VI_PIPE_INT_CTL
{
    /* Define the struct bits */
    struct
    {
        unsigned int    win_ctl_frame_end_int          : 1   ; /* [0 ]      */
        unsigned int    emb_ctl_frame_end_int          : 1   ; /* [1 ]      */
        unsigned int    emb_ctl_time_out_int           : 1   ; /* [2 ]      */
        unsigned int    tpg_ctl_frame_end_int          : 1   ; /* [3 ]      */
        unsigned int    dma_y_wr_frame_end_int         : 1   ; /* [4 ]      */
        unsigned int    dma_y_wr_ch_line_base_int      : 1   ; /* [5 ]      */
        unsigned int    dma_y_wr_ch_err_frame_end_int  : 1   ; /* [6 ]      */
        unsigned int    dma_y_wr_ch_err_immediate_int  : 1   ; /* [7 ]      */
        unsigned int    dma_uv_wr_ch_frame_end_int     : 1   ; /* [8 ]      */
        unsigned int    dma_uv_wr_ch_line_base_int     : 1   ; /* [9 ]      */
        unsigned int    dma_uv_wr_ch_err_frame_end_int : 1   ; /* [10]      */
        unsigned int    dma_uv_wr_ch_err_immediate_int : 1   ; /* [11]      */
        unsigned int    dma_raw_rd_ch_frame_end_int    : 1   ; /* [12]      */
        unsigned int    dma_raw_rd_ch_line_base_int    : 1   ; /* [13]      */
        unsigned int    dma_raw_rd_ch_err_frame_end_int : 1   ; /* [14]      */
        unsigned int    dma_raw_rd_ch_err_immediate_int : 1   ; /* [15]      */
        unsigned int    win_ctl_frame_end_mask         : 1   ; /* [16]      */
        unsigned int    emb_ctl_frame_end_mask         : 1   ; /* [17]      */
        unsigned int    emb_ctl_time_out_mask          : 1   ; /* [18]      */
        unsigned int    tpg_ctl_frame_end_mask         : 1   ; /* [19]      */
        unsigned int    dma_y_wr_frame_end_mask        : 1   ; /* [20]      */
        unsigned int    dma_y_wr_ch_line_base_mask     : 1   ; /* [21]      */
        unsigned int    dma_y_wr_ch_err_frame_end_mask : 1   ; /* [22]      */
        unsigned int    dma_y_wr_ch_err_immediate_mask : 1   ; /* [23]      */
        unsigned int    dma_uv_wr_ch_frame_end_mask    : 1   ; /* [24]      */
        unsigned int    dma_uv_wr_ch_line_base_mask    : 1   ; /* [25]      */
        unsigned int    dma_uv_wr_ch_err_frame_end_mask : 1   ; /* [26]      */
        unsigned int    dma_uv_wr_ch_err_immediate_mask : 1   ; /* [27]      */
        unsigned int    dma_raw_rd_ch_frame_end_mask   : 1   ; /* [28]      */
        unsigned int    dma_raw_rd_ch_line_base_mask   : 1   ; /* [29]      */
        unsigned int    dma_raw_rd_ch_err_frame_end_mask : 1   ; /* [30]      */
        unsigned int    dma_raw_rd_ch_err_immediate_mask : 1   ; /* [31]      */
    } bits;    /* Define an unsigned member */

    unsigned int    u32;
} ;

#define  VI_PIPE_CTRL     				(0x0000)
#define  VI_PIPE_TIMING_CTRL    		(0x0004)
#define  VI_PIPE_SYNC_WIDTH     		(0x0008)
#define  VI_PIPE_IMAGE_SIZE_TOTAL   	(0x000C)
#define  VI_PIPE_IMAGE_SIZE_ACTIVE  	(0x0010)
#define  VI_PIPE_IMAGE_START     		(0x0014)
	
#define  VI_PIPE_FRM_BASE_ADDR0_Y   	(0x0020)
#define  VI_PIPE_FRM_BASE_ADDR1_Y   	(0x0024)
#define  VI_PIPE_FRM_BASE_ADDR0_UV  	(0x0028)
#define  VI_PIPE_FRM_BASE_ADDR1_UV  	(0x002C)
#define  VI_PIPE_LINE_STRIDE     		(0x0030)
	
#define  VI_PIPE_TPG_DMA_CTL     		(0x0040)
#define  VI_PIPE_TPG_CTL     			(0x0044)
#define  VI_PIPE_TPG_TOF     			(0x0048)
#define  VI_PIPE_TPG_SIZE_TOTAL     	(0x004C)
#define  VI_PIPE_TPG_SIZE_ACTIVE    	(0x0050)
#define  VI_PIPE_TPG_START     			(0x0054)
#define  VI_PIPE_TPG_FRM_BASE_ADDR0_Y   (0x0058)
#define  VI_PIPE_TPG_FRM_BASE_ADDR1_Y   (0x005C)
#define  VI_PIPE_TPG_LINE_STRIDE     	(0x0060)

#define  VI_PIPE_DMA_MODE     			(0x006C)
#define  VI_PIPE_DMA_ERROR_MODE     	(0x0070)
#define  VI_PIPE_DMA_RST_REQ     		(0x0074)
#define  VI_PIPE_CONFIG_DONE     		(0x0078)
#define  VI_PIPE_INT_CTL     			(0x007C)

#endif  /*_VI_PIPE_REG_H_*/

