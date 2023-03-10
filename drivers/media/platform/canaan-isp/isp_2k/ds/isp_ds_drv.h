/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef _ISP_DS_DRV_H_
#define _ISP_DS_DRV_H_

#include "../../k510isp_com.h"
#include "isp_ds_reg.h"
#include "isp_ds_drv.h"

typedef struct _ISP_DS_VSCALE_PARM_S
{
    unsigned int   scale_en;     
//     
    unsigned int   vscale_filter_en;  
    unsigned int   vscale_dstep;    
    unsigned int   vscale_pstep;           
} ISP_DS_VSCALE_PARM_S;

typedef struct _ISP_DS_HSCALE_PARM_S
{
//
    unsigned int   hscale_filter_en;   
    unsigned int   hscale_dstep;     
    unsigned int   hscale_pstep;
} ISP_DS_HSCALE_PARM_S;

typedef struct _ISP_DS_SIZE_S
{
    unsigned int    frame_output_height;
    unsigned int    frame_output_width; 
} ISP_DS_SIZE_S;

typedef struct _ISP_DS_FORMAT_S
{
    unsigned int    out_rgb_mode;
    unsigned int    out_rgb_en;  
    unsigned int    out_yuv_mode;
    unsigned int    out_uv_swap;       
} ISP_DS_FORMAT_S;

typedef struct _ISP_DS_OSD_INFO_S
{
    unsigned int    osd_type;      
    unsigned int    osd_alpha_tpye;
    unsigned int    osd_enable;          
} ISP_DS_OSD_INFO_S;

typedef struct _ISP_DS_OSD_BUF_S
{
    unsigned int    osd_rgb_addr0;    
    unsigned int    osd_alpha_addr0;  
    unsigned int    osd_rgb_addr1;    
    unsigned int    osd_alpha_addr1;  

    unsigned int    osd_stride;       
    unsigned int    osd_alpha_stride; 

    unsigned int    osd_position_start_x;
    unsigned int    osd_position_stop_x;      

    unsigned int    osd_position_start_y;
    unsigned int    osd_position_stop_y;      
}ISP_DS_OSD_BUF_S;

typedef struct _ISP_DS_OSD_DMA_CTL_S
{
    unsigned int    osd_dma_request_length;
    unsigned int    osd_dma_map;           
    unsigned int    osd_rgb_rev;           
    unsigned int    osd_global_alpha;       
    unsigned int    osd_swap_64;
    unsigned int    osd_outstanding_num;
    unsigned int    osd_bd_limit_en;
} ISP_DS_OSD_DMA_CTL_S;

typedef struct _ISP_DS_OSD_ATTR_S
{
    ISP_DS_OSD_INFO_S 		OsdInfo;
    IMAGE_SIZE 				OsdSize;
    ISP_DS_OSD_BUF_S 		OsdBuf;
    ISP_DS_OSD_DMA_CTL_S 	OsdDmaCtl;
} ISP_DS_OSD_ATTR_S;

typedef struct _ISP_S_DS_ATTR_S
{
    IMAGE_SIZE           dsOutSize;
    ISP_DS_VSCALE_PARM_S vscalePram;
    ISP_DS_HSCALE_PARM_S hscalePram;
    ISP_DS_FORMAT_S      DsFormat;
    ISP_DS_OSD_ATTR_S    DsOsdAttr[ISP_DS_CH_OSD_NUM];
} ISP_S_DS_ATTR_S;

typedef struct _ISP_DS_ATTR_S
{
	IMAGE_SIZE input_size;
	unsigned int osd_rgb2yuv_coeff[3][4];
	unsigned int osd_yuv2rgb_coeff[3][4];
	ISP_S_DS_ATTR_S isp_ds_attr[ISP_DS_CH_NUM];
} ISP_DS_ATTR_S;
/*
*f2k
*/
void Isp_Drv_F2k_Ds_SetInputSize(struct k510_isp_device *isp,IMAGE_SIZE *inputsize);
void Isp_Drv_F2k_Ds_SetOutputSize(struct k510_isp_device *isp,unsigned char  Index, IMAGE_SIZE *OutputSize);
void Isp_Drv_F2k_Ds_SetRgb2YuvCoff(struct k510_isp_device *isp,unsigned int *osd_rgb2yuv_coeff);
void Isp_Drv_F2k_Ds_SetYuv2RgbCoff(struct k510_isp_device *isp,unsigned int *osd_yuv2rgb_coeff);
void Isp_Drv_F2k_Ds_SetOutputFormat(struct k510_isp_device *isp,unsigned char  Index, ISP_DS_FORMAT_S *Format);
void Isp_Drv_F2k_Ds_SetscaleCoeff(struct k510_isp_device *isp);
void Isp_Drv_F2k_Ds_SetVCoef(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_F2k_Ds_SetHCoef0(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_F2k_Ds_SetHCoef1(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_F2k_Ds_SetHCoef2(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_F2k_Ds_SetSingleDS(struct k510_isp_device *isp,unsigned char  Index,ISP_S_DS_ATTR_S *sDsAttr);
/*
*r2k
*/
void Isp_Drv_R2k_Ds_SetInputSize(struct k510_isp_device *isp,IMAGE_SIZE *inputsize);
void Isp_Drv_R2k_Ds_SetOutputSize(struct k510_isp_device *isp,unsigned char  Index, IMAGE_SIZE *OutputSize);
void Isp_Drv_R2k_Ds_SetRgb2YuvCoff(struct k510_isp_device *isp,unsigned int *osd_rgb2yuv_coeff);
void Isp_Drv_R2k_Ds_SetYuv2RgbCoff(struct k510_isp_device *isp,unsigned int *osd_yuv2rgb_coeff);
void Isp_Drv_R2k_Ds_SetOutputFormat(struct k510_isp_device *isp,unsigned char  Index, ISP_DS_FORMAT_S *Format);
void Isp_Drv_R2k_Ds_SetscaleCoeff(struct k510_isp_device *isp);
void Isp_Drv_R2k_Ds_SetVCoef(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_R2k_Ds_SetHCoef0(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_R2k_Ds_SetHCoef1(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_R2k_Ds_SetHCoef2(struct k510_isp_device *isp,int a[],int num);
void Isp_Drv_R2k_Ds_SetSingleDS(struct k510_isp_device *isp,unsigned char  Index,ISP_S_DS_ATTR_S *sDsAttr);
#endif /*_ISP_DS_DRV_H_*/
