/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
//#include "../../k510isp_com.h"
#include "../../k510_isp.h"
#include "isp_core_drv.h"
#include "isp_core_reg.h"
/*
*  Image Timing Control
*/
int Isp_Drv_F2k_Core_SetItcCtl(struct k510_isp_device *isp,ISP_CORE_ITC_CTL_S *pstItcCtl)
{

	union U_ISP_CORE_ITC_CTL stData;
	stData.u32 = 0;
	stData.bits.hsync_pol = pstItcCtl->hsync_pol;
	stData.bits.vsync_pol = pstItcCtl->vsync_pol;
	stData.bits.hsync_input_timing = pstItcCtl->hsync_input_timing;
	stData.bits.vsync_input_timing = pstItcCtl->vsync_input_timing;
	stData.bits.mirror_ctl = pstItcCtl->mirror_ctl;
	stData.bits.video_format_ctl = pstItcCtl->video_format_ctl;

	isp_reg_writel(isp,stData.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ITC_CTL);
	return 0;
}
/*
*input image size
*/
int Isp_Drv_F2k_Core_SetImageAttr(struct k510_isp_device *isp,ISP_CORE_IMAGE_ATTR_S *pstImgAttr)
{


	union U_ISP_CORE_IMAGE_HEIGHT stHeight;
	stHeight.u32 = 0;
	stHeight.bits.image_height = pstImgAttr->image_height;
	isp_reg_writel(isp,stHeight.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_HEIGHT);

	union U_ISP_CORE_IMAGE_WIDTH stWidth;
	stWidth.u32 = 0;
	stWidth.bits.image_width = pstImgAttr->image_width;
	isp_reg_writel(isp,stWidth.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_WIDTH);

	union U_ISP_CORE_IMAGE_V_START stVStart;
	stVStart.u32 = 0;
	stVStart.bits.image_v_start = pstImgAttr->image_v_start;
	isp_reg_writel(isp,stVStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_V_START);

	union U_ISP_CORE_IMAGE_H_START stHStart;
	stHStart.u32 = 0;
	stHStart.bits.image_h_start = pstImgAttr->image_h_start;
	isp_reg_writel(isp,stHStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_H_START);

	union U_ISP_CORE_IMAGE_ACTIVE_WIDTH stActWidth;
	stActWidth.u32 = 0;
	stActWidth.bits.image_active_width = pstImgAttr->image_active_width;
	isp_reg_writel(isp,stActWidth.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_ACTIVE_WIDTH);

	union U_ISP_CORE_IMAGE_ACTIVE_HEIGHT stActHeight;
	stActHeight.u32 = 0;
	stActHeight.bits.image_active_height = pstImgAttr->image_active_height;
	isp_reg_writel(isp,stActHeight.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_IMAGE_ACTIVE_HEIGHT);
	return 0;
}
/*
*Test Pattern Control
*/
int Isp_Drv_F2k_Core_SetTestCtl(struct k510_isp_device *isp,ISP_CORE_TEST_CTL_S *pstTestCtl)
{

 	union U_ISP_CORE_TEST_CTL stData;
	stData.u32 = 0;
	stData.bits.bayer_mode_sel = pstTestCtl->bayer_mode_sel;
	stData.bits.motion_mode_sel = pstTestCtl->motion_mode_sel;
	stData.bits.test_pattern_sel = pstTestCtl->test_pattern_sel;
	stData.bits.test_pattern_en = pstTestCtl->test_pattern_en;
	stData.bits.wdr_l_mul_data = pstTestCtl->wdr_l_mul_data;
	stData.bits.wdr_m_mul_data = pstTestCtl->wdr_m_mul_data;
	stData.bits.wdr_s_mul_data = pstTestCtl->wdr_s_mul_data;
	isp_reg_writel(isp,stData.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_TEST_CTL);
	return 0;
}
/*
*Black Level Correction
*/
int Isp_Drv_F2k_Core_SetBlcCtl(struct k510_isp_device *isp,ISP_CORE_BLC_CTL_S *pstBlcCtl)
{

	union U_ISP_CORE_BLC_CTL stBlcCtl;
	stBlcCtl.u32 = 0;
	stBlcCtl.bits.blc_en = pstBlcCtl->blc_en;
	isp_reg_writel(isp,stBlcCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_BLC_CTL);

	union U_ISP_CORE_BLC_OFFSET stBlcOffset;
	stBlcOffset.u32 = 0;
	stBlcOffset.bits.blc_offset = pstBlcCtl->blc_offset;
	isp_reg_writel(isp,stBlcOffset.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_BLC_OFFSET);

	union U_ISP_CORE_BLC_RATIO stBlcRatio;
	stBlcRatio.u32 = 0;
	stBlcRatio.bits.blc_ratio = pstBlcCtl->blc_ratio;
	isp_reg_writel(isp,stBlcRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_BLC_RATIO);
	return 0;
}
/*
* Lens Shading Correction
*/
int Isp_Drv_F2k_Core_SetLscCtl(struct k510_isp_device *isp,ISP_CORE_LSC_CTL_S *pstLscCtl)
{

	union U_ISP_CORE_LSC_CTL stlscCtl;
	stlscCtl.u32 = 0;
	stlscCtl.bits.lsc_en = pstLscCtl->lsc_en;
	isp_reg_writel(isp,stlscCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_CTL);

	union U_ISP_CORE_LSC_H_CENTER stHCenter;
	stHCenter.u32 = 0;
	stHCenter.bits.lsc_h_center = pstLscCtl->lsc_h_center;
	isp_reg_writel(isp,stHCenter.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_H_CENTER);

	union U_ISP_CORE_LSC_V_CENTER stVCenter;
	stVCenter.u32 = 0;
	stVCenter.bits.lsc_v_center = pstLscCtl->lsc_v_center;
	isp_reg_writel(isp,stVCenter.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_V_CENTER);

	union U_ISP_CORE_LSC_R_RATIO stRRatio;
	stRRatio.u32 = 0;
	stRRatio.bits.lsc_red_ratio = pstLscCtl->lsc_red_ratio;
	isp_reg_writel(isp,stRRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_R_RATIO);

	union U_ISP_CORE_LSC_G_RATIO stGRatio;
	stGRatio.u32 = 0;
	stGRatio.bits.lsc_green_ratio = pstLscCtl->lsc_green_ratio;
	isp_reg_writel(isp,stGRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_G_RATIO);

	union U_ISP_CORE_LSC_B_RATIO stBRatio;
	stBRatio.u32 = 0;
	stBRatio.bits.lsc_blue_ratio = pstLscCtl->lsc_blue_ratio;
	isp_reg_writel(isp,stBRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_B_RATIO);

	union U_ISP_CORE_LSC_IR_RATIO stIrRatio;
	stIrRatio.u32 = 0;
	stIrRatio.bits.lsc_ir_ratio = pstLscCtl->lsc_ir_ratio;
	isp_reg_writel(isp,stIrRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LSC_IR_RATIO);

	return 0;
}
/*
*AUTO Exposure/Gain
*/
int Isp_Drv_F2k_Core_SetAeCtl(struct k510_isp_device *isp,ISP_CORE_AE_CTL_S *pstAeCtl)
{

	union U_ISP_CORE_AE_CTL stAeCtl;
	stAeCtl.u32 = 0;
	stAeCtl.bits.ae_as_en = pstAeCtl->ae_as_en;
	stAeCtl.bits.ae_ag_en = pstAeCtl->ae_ag_en;
	stAeCtl.bits.ae_airis_en = pstAeCtl->ae_airis_en;
	stAeCtl.bits.ae_enter_ls_sel = pstAeCtl->ae_enter_ls_sel;
	stAeCtl.bits.ae_exit_ls_sel = pstAeCtl->ae_exit_ls_sel;
	stAeCtl.bits.ae_win_mode_sel = pstAeCtl->ae_win_mode_sel;
	stAeCtl.bits.ae_back_light_mode_sel = pstAeCtl->ae_back_light_mode_sel;
	stAeCtl.bits.ae_day_change_en = pstAeCtl->ae_day_change_en;
	stAeCtl.bits.ae_day_change_sel = pstAeCtl->ae_day_change_sel;
	isp_reg_writel(isp,stAeCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_CTL);

	union U_ISP_CORE_AE_WIN_H_START stHStart;
	stHStart.u32 = 0;
	stHStart.bits.ae_win_h_start = pstAeCtl->ae_win_h_start;
	isp_reg_writel(isp,stHStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_WIN_H_START);

	union U_ISP_CORE_AE_WIN_V_START stVStart;
	stVStart.u32 = 0;
	stVStart.bits.ae_win_v_start = pstAeCtl->ae_win_v_start;
	isp_reg_writel(isp,stVStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_WIN_V_START);

	union U_ISP_CORE_AE_WIN_H_END stHEnd;
	stHEnd.u32 = 0;
	stHEnd.bits.ae_win_h_end = pstAeCtl->ae_win_h_end;
	isp_reg_writel(isp,stHEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_WIN_H_END);

	union U_ISP_CORE_AE_WIN_V_END stVEnd;
	stVEnd.u32 = 0;
	stVEnd.bits.ae_win_v_end = pstAeCtl->ae_win_v_end;
	isp_reg_writel(isp,stVEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_WIN_V_END);

	union U_ISP_CORE_AE_TAR_BR stTarBr;
	stTarBr.u32 = 0;
	stTarBr.bits.ae_tar_bright = pstAeCtl->ae_tar_bright;
	isp_reg_writel(isp,stTarBr.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_TAR_BR);

	union U_ISP_CORE_AE_TAR_BR_RANGE stTarBrRange;
	stTarBrRange.u32 = 0;
	stTarBrRange.bits.ae_tar_bright_range = pstAeCtl->ae_tar_bright_range;
	isp_reg_writel(isp,stTarBrRange.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_TAR_BR_RANGE);

	union U_ISP_CORE_AE_L_EX_TIME stLExTime;
	stLExTime.u32 = 0;
	stLExTime.bits.ae_l_ex_time = pstAeCtl->ae_l_ex_time;
	isp_reg_writel(isp,stLExTime.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_L_EX_TIME);

	union U_ISP_CORE_AE_M_EX_TIME stMExTime;
	stMExTime.u32 = 0;
	stMExTime.bits.ae_m_ex_time = pstAeCtl->ae_m_ex_time;
	isp_reg_writel(isp,stMExTime.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_M_EX_TIME);

	union U_ISP_CORE_AE_S_EX_TIME stSExTime;
	stSExTime.u32 = 0;
	stSExTime.bits.ae_s_ex_time = pstAeCtl->ae_s_ex_time;
	isp_reg_writel(isp,stSExTime.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_S_EX_TIME);

	union U_ISP_CORE_AE_AGC stAgc;
	stAgc.u32 = 0;
	stAgc.bits.ae_agc = pstAeCtl->ae_agc;
	isp_reg_writel(isp,stAgc.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_AGC);

	union U_ISP_CORE_AE_ADJUST_CTL stAdCtl;
	stAdCtl.u32 = 0;
	stAdCtl.bits.ae_ad_shuttle_freq = pstAeCtl->ae_ad_shuttle_freq;
	stAdCtl.bits.ae_ad_gain_freq = pstAeCtl->ae_ad_gain_freq;
	isp_reg_writel(isp,stAdCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_ADJUST_CTL);

	union U_ISP_CORE_AE_ADJUST_STEP_MAX stAdStepMax;
	stAdStepMax.u32 = 0;
	stAdStepMax.bits.ae_adjust_step_max = pstAeCtl->ae_adjust_step_max;
	isp_reg_writel(isp,stAdStepMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_ADJUST_STEP_MAX);

    union U_ISP_CORE_AE_EX_VALUE_MAX stExValueMax;
	stExValueMax.u32 = 0;
	stExValueMax.bits.ae_ex_value_max = pstAeCtl->ae_ex_value_max;
	isp_reg_writel(isp,stExValueMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_EX_VALUE_MAX);

	union U_ISP_CORE_AE_EX_VALUE_MID stExValueMid;
	stExValueMid.u32 = 0;
	stExValueMid.bits.ae_ex_value_mid = pstAeCtl->ae_ex_value_mid;
	isp_reg_writel(isp,stExValueMid.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_EX_VALUE_MID);

	union U_ISP_CORE_AE_EX_VALUE_MIN stExValueMin;
	stExValueMin.u32 = 0;
	stExValueMin.bits.ae_ex_value_min = pstAeCtl->ae_ex_value_min;
	isp_reg_writel(isp,stExValueMin.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_EX_VALUE_MIN);

    union U_ISP_CORE_AE_GAIN_MAX stGainMax;
	stGainMax.u32 = 0;
	stGainMax.bits.ae_gain_value_max = pstAeCtl->ae_gain_value_max;
	isp_reg_writel(isp,stGainMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_GAIN_MAX);

	union U_ISP_CORE_AE_GAIN_MID stGainMid;
	stGainMid.u32 = 0;
	stGainMid.bits.ae_gain_value_mid = pstAeCtl->ae_gain_value_mid;
	isp_reg_writel(isp,stGainMid.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_GAIN_MID);

	union U_ISP_CORE_AE_GAIN_MIN stGainMin;
	stGainMin.u32 = 0;
	stGainMin.bits.ae_gain_value_min = pstAeCtl->ae_gain_value_min;
	isp_reg_writel(isp,stGainMin.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_GAIN_MIN);

	union U_ISP_CORE_AE_DN_SWITCH_ADJUST_STEP_MAX stDnAdStepMax;
	stDnAdStepMax.u32 = 0;
	stDnAdStepMax.bits.ae_dn_switch_ad_step_max = pstAeCtl->ae_dn_switch_ad_step_max;
	isp_reg_writel(isp,stDnAdStepMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_DN_SWITCH_ADJUST_STEP_MAX);

	union U_ISP_CORE_AE_DN_SWITCH_WAIT_TIME stDnWaitTime;
	stDnWaitTime.u32 = 0;
	stDnWaitTime.bits.ae_dn_switch_wait_time = pstAeCtl->ae_dn_switch_wait_time;
	isp_reg_writel(isp,stDnWaitTime.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_DN_SWITCH_WAIT_TIME);

 	union U_ISP_CORE_APE_DIFF_MAX stApeDiffMax;
	stApeDiffMax.u32 = 0;
	stApeDiffMax.bits.ape_max_diff = pstAeCtl->ape_max_diff;
	isp_reg_writel(isp,stApeDiffMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_DIFF_MAX);

	union U_ISP_CORE_APE_DRV_SIGNAL_MAX stDriveSignalMax;
	stDriveSignalMax.u32 = 0;
	stDriveSignalMax.bits.ape_drv_signal_max = pstAeCtl->ape_drv_signal_max;
	isp_reg_writel(isp,stDriveSignalMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_DRV_SIGNAL_MAX);

	union U_ISP_CORE_APE_COEFF_DIS stApeCoeffDis;
	stApeCoeffDis.u32 = 0;
	stApeCoeffDis.bits.ape_coeff_distance = pstAeCtl->ape_coeff_distance;
	isp_reg_writel(isp,stApeCoeffDis.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_COEFF_DIS);

	union U_ISP_CORE_APE_COEFF_SPEED stApeCoeffSpeed;
	stApeCoeffSpeed.u32 = 0;
	stApeCoeffSpeed.bits.ape_coeff_speed = pstAeCtl->ape_coeff_speed;
	isp_reg_writel(isp,stApeCoeffSpeed.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_COEFF_SPEED);

	union U_ISP_CORE_APE_COEFF_ACCE stApeCoeffAcce;
	stApeCoeffAcce.u32 = 0;
	stApeCoeffAcce.bits.ape_coeff_acceleration = pstAeCtl->ape_coeff_acceleration;
	isp_reg_writel(isp,stApeCoeffAcce.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_COEFF_ACCE);

	union U_ISP_CORE_APE_DRV_MANUAL_VALUE stApeDriManValue;
	stApeDriManValue.u32 = 0;
	stApeDriManValue.bits.ape_drv_manual_value = pstAeCtl->ape_drv_manual_value;
	isp_reg_writel(isp,stApeDriManValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_DRV_MANUAL_VALUE);

	union U_ISP_CORE_APE_DAMP_MANUAL_VALUE stDampManValue;
	stDampManValue.u32 = 0;
	stDampManValue.bits.ape_damp_manual_value = pstAeCtl->ape_damp_manual_value;
	isp_reg_writel(isp,stDampManValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_APE_DAMP_MANUAL_VALUE);
	return 0;
}
/**
 * @brief
 *
 * @return int
 */
int Isp_Drv_F2k_Core_GetAeLongCurEx(struct k510_isp_device *isp)
{
	int data;


	data = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_LONG_CUR_EX);

	return data;
}
/**
 * @brief
 *
 * @return int
 */
int Isp_Drv_F2k_Core_GetAeCurDigitalGain(struct k510_isp_device *isp)
{
	int data;


	data = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_CUR_DIGITAL_GAIN);

	return data;
}
/**
 * @brief
 *
 * @param pAeSts
 * @return int
 */
int Isp_Drv_F2k_Core_GetAeSts(struct k510_isp_device *isp,ISP_CORE_AE_STS_S *pAeSts)
{

  	union U_ISP_CORE_AE_VALUE_READY          gtAeValueReady;
  	gtAeValueReady.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_VALUE_READY);
  	pAeSts->ae_value_ready = gtAeValueReady.u32;

  	union U_ISP_CORE_AE_LONG_CUR_EX          gtAeLongCurEX;
  	gtAeLongCurEX.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_LONG_CUR_EX);
  	pAeSts->ae_long_cur_ex = gtAeLongCurEX.u32;

  	union U_ISP_CORE_AE_MID_CUR_EX           gtAeMidCurEX;
  	gtAeMidCurEX.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_MID_CUR_EX);
  	pAeSts->ae_mid_cur_ex =  gtAeMidCurEX.u32;

  	union U_ISP_CORE_AE_SHORT_CUR_EX         gtAeShortCurEX;
  	gtAeShortCurEX.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_SHORT_CUR_EX);
  	pAeSts->ae_short_cur_ex =  gtAeShortCurEX.u32;

  	union U_ISP_CORE_AE_CUR_DIGITAL_GAIN     gtAeCurDigitalGain;
  	gtAeCurDigitalGain.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_CUR_DIGITAL_GAIN);
  	pAeSts->ae_digital_gain =  gtAeCurDigitalGain.u32;

  	union U_ISP_CORE_AE_CUR_AVE_BRIGHTNESS   gtAeCurAvBrightness;
  	gtAeCurAvBrightness.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_CUR_AVE_BRIGHTNESS);
  	pAeSts->ave_brightness =  gtAeCurAvBrightness.u32;

  	union U_ISP_CORE_AE_CUR_DN_STATUS        gtAeCurDNStatus;
  	gtAeCurDNStatus.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_CUR_DN_STATUS);
  	pAeSts->ae_day_night_status =  gtAeCurDNStatus.u32;

  	union U_ISP_CORE_AE_EX_STATUS            gtAeEXStatus;
  	gtAeEXStatus.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_EX_STATUS);
  	pAeSts->ae_ex_status =  gtAeEXStatus.u32;

  	union U_ISP_CORE_AE_SUM                  gtAgtAeShortCurEXeSum;
  	gtAgtAeShortCurEXeSum.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_SUM);
  	pAeSts->ae_sum =  gtAgtAeShortCurEXeSum.u32;

  	union U_ISP_CORE_AE_PIXEL_SUM            gtAePixelSum;
  	gtAePixelSum.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AE_PIXEL_SUM);
  	pAeSts->ae_mid_cur_ex =  gtAePixelSum.u32;

  	return 0;
}
/*
*AWB
*/
int Isp_Drv_F2k_Core_SetAwbCtl(struct k510_isp_device *isp,ISP_CORE_AWB_CTL_S *pstAwbCtl)
{

	union U_ISP_CORE_AWB_CTL stAwbCtl;
	stAwbCtl.u32 = 0;
	stAwbCtl.bits.awb_d65_en = pstAwbCtl->awb_d65_en;
	stAwbCtl.bits.awb_ccm_en = pstAwbCtl->awb_ccm_en;
	stAwbCtl.bits.awb_en = pstAwbCtl->awb_en;
	stAwbCtl.bits.awb_mode_sel = pstAwbCtl->awb_mode_sel;
	stAwbCtl.bits.awb_hist_mode_sel = pstAwbCtl->awb_hist_mode_sel;
	stAwbCtl.bits.awb_veri_en = pstAwbCtl->awb_veri_en;
	stAwbCtl.bits.awb_fb_en = pstAwbCtl->awb_fb_en;
	stAwbCtl.bits.awb_value_save_en = pstAwbCtl->awb_value_save_en;
	stAwbCtl.bits.awb_ccm_adp_adjust_en = pstAwbCtl->awb_ccm_adp_adjust_en;
	stAwbCtl.bits.awb_stab_en = pstAwbCtl->awb_stab_en;
	isp_reg_writel(isp,stAwbCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_CTL);

	union U_ISP_CORE_AWB_D65_RED_GAIN stD65RedGain;
	stD65RedGain.u32 = 0;
	stD65RedGain.bits.awb_d65_red_gain = pstAwbCtl->awb_d65_red_gain;
	isp_reg_writel(isp,stD65RedGain.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_D65_RED_GAIN);

	union U_ISP_CORE_AWB_D65_BLUE_GAIN stD65BlueGain;
	stD65BlueGain.u32 = 0;
	stD65BlueGain.bits.awb_d65_blue_gain = pstAwbCtl->awb_d65_blue_gain;
	isp_reg_writel(isp,stD65BlueGain.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_D65_BLUE_GAIN);

 	union U_ISP_CORE_CCM_RR_COFF stRrCoff;
	stRrCoff.u32 = 0;
	stRrCoff.bits.ccm_rr_coff = pstAwbCtl->ccm_rr_coff;
	isp_reg_writel(isp,stRrCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_RR_COFF);

	union U_ISP_CORE_CCM_RG_COFF stRgCoff;
	stRgCoff.u32 = 0;
	stRgCoff.bits.ccm_rg_coff = pstAwbCtl->ccm_rg_coff;
	isp_reg_writel(isp,stRgCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_RG_COFF);

	union U_ISP_CORE_CCM_RB_COFF stRboff;
	stRboff.u32 = 0;
	stRboff.bits.ccm_rb_coff = pstAwbCtl->ccm_rb_coff;
    isp_reg_writel(isp,stRboff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_RB_COFF);

	union U_ISP_CORE_CCM_GR_COFF stGrCoff;
	stGrCoff.u32 = 0;
	stGrCoff.bits.ccm_gr_coff = pstAwbCtl->ccm_gr_coff;
	isp_reg_writel(isp,stGrCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_GR_COFF);

	union U_ISP_CORE_CCM_GG_COFF stGgCoff;
	stGgCoff.u32 = 0;
	stGgCoff.bits.ccm_gg_coff = pstAwbCtl->ccm_gg_coff;
	isp_reg_writel(isp,stGgCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_GG_COFF);

	union U_ISP_CORE_CCM_GB_COFF stGbCoff;
	stGbCoff.u32 = 0;
	stGbCoff.bits.ccm_gb_coff = pstAwbCtl->ccm_gb_coff;
	isp_reg_writel(isp,stGbCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_GB_COFF);

	union U_ISP_CORE_CCM_BR_COFF stBrCoff;
	stBrCoff.u32 = 0;
	stBrCoff.bits.ccm_br_coff = pstAwbCtl->ccm_br_coff;
	isp_reg_writel(isp,stBrCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_BR_COFF);

	union U_ISP_CORE_CCM_BG_COFF stBgCoff;
	stBgCoff.u32 = 0;
	stBgCoff.bits.ccm_bg_coff = pstAwbCtl->ccm_bg_coff;
	isp_reg_writel(isp,stBgCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_BG_COFF);

	union U_ISP_CORE_CCM_BB_COFF stBbCoff;
	stBbCoff.u32 = 0;
	stBbCoff.bits.ccm_bb_coff = pstAwbCtl->ccm_bb_coff;
	isp_reg_writel(isp,stBbCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_BB_COFF);

 	union U_ISP_CORE_CCM_CORRECT_COFF stCorrCoff;
	stCorrCoff.u32 = 0;
	stCorrCoff.bits.ccm_correct_coff = pstAwbCtl->ccm_correct_coff;
	isp_reg_writel(isp,stCorrCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CCM_CORRECT_COFF);

	union U_ISP_CORE_AWB_WIN_H_START stHStart;
	stHStart.u32 = 0;
	stHStart.bits.awb_win_h_start = pstAwbCtl->awb_win_h_start;
	isp_reg_writel(isp,stHStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_WIN_H_START);

    union U_ISP_CORE_AWB_WIN_V_START stVStart;
	stVStart.u32 = 0;
	stVStart.bits.awb_win_v_start = pstAwbCtl->awb_win_v_start;
	isp_reg_writel(isp,stVStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_WIN_V_START);

	union U_ISP_CORE_AWB_WIN_H_END stHEnd;
	stHEnd.u32 = 0;
	stHEnd.bits.awb_win_h_end = pstAwbCtl->awb_win_h_end;
	isp_reg_writel(isp,stHEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_WIN_H_END);

	union U_ISP_CORE_AWB_WIN_V_END stVEnd;
	stVEnd.u32 = 0;
	stVEnd.bits.awb_win_v_end = pstAwbCtl->awb_win_v_end;
	isp_reg_writel(isp,stVEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_WIN_V_END);

    union U_ISP_CORE_AWB_CORRECT_DIFF_TH stCorDiffTh;
	stCorDiffTh.u32 = 0;
	stCorDiffTh.bits.awb_correct_diff_th = pstAwbCtl->awb_correct_diff_th;
	isp_reg_writel(isp, stCorDiffTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_CORRECT_DIFF_TH);

	union U_ISP_CORE_AWB_RES_TIME stResTime;
	stResTime.u32 = 0;
	stResTime.bits.awb_color_changeres_time = pstAwbCtl->awb_color_changeres_time;
	isp_reg_writel(isp, stResTime.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RES_TIME);

 	union U_ISP_CORE_AWB_HIST_TH stHistTh;
	stHistTh.u32 = 0;
	stHistTh.bits.awb_historgram_th = pstAwbCtl->awb_historgram_th;
	isp_reg_writel(isp, stHistTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_HIST_TH);

	union U_ISP_CORE_AWB_RED_GAIN_ADJUST stRedGainAd;
	stRedGainAd.u32 = 0;
	stRedGainAd.bits.awb_red_gain_adjust = pstAwbCtl->awb_red_gain_adjust;
	isp_reg_writel(isp, stRedGainAd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_GAIN_ADJUST);

	union U_ISP_CORE_AWB_GREEN_GAIN_ADJUST stGreenGainAd;
	stGreenGainAd.u32 = 0;
	stGreenGainAd.bits.awb_green_gain_adjust = pstAwbCtl->awb_green_gain_adjust;
	isp_reg_writel(isp, stGreenGainAd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_GREEN_GAIN_ADJUST);

	union U_ISP_CORE_AWB_BLUE_GAIN_ADJUST stBlueGainAd;
	stBlueGainAd.u32 = 0;
	stBlueGainAd.bits.awb_blue_gain_adjust = pstAwbCtl->awb_blue_gain_adjust;
	isp_reg_writel(isp, stBlueGainAd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_GAIN_ADJUST);

	union U_ISP_CORE_AWB_RED_MAX_VALUE stRedMaxValue;
	stRedMaxValue.u32 = 0;
	stRedMaxValue.bits.awb_red_max_value = pstAwbCtl->awb_red_max_value;
	isp_reg_writel(isp, stRedMaxValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_MAX_VALUE);

	union U_ISP_CORE_AWB_BLUE_MAX_VALUE stBlueMaxValue;
	stBlueMaxValue.u32 = 0;
	stBlueMaxValue.bits.awb_blue_max_value = pstAwbCtl->awb_blue_max_value;
	isp_reg_writel(isp, stBlueMaxValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_MAX_VALUE);

	union U_ISP_CORE_AWB_RED_MIN_VALUE stRedMinValue;
	stRedMinValue.u32 = 0;
	stRedMinValue.bits.awb_red_min_value = pstAwbCtl->awb_red_min_value;
	isp_reg_writel(isp, stRedMinValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_MIN_VALUE);

	union U_ISP_CORE_AWB_BLUE_MIN_VALUE stBlueMinValue;
	stBlueMinValue.u32 = 0;
	stBlueMinValue.bits.awb_blue_min_value = pstAwbCtl->awb_blue_min_value;
	isp_reg_writel(isp, stBlueMinValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_MIN_VALUE);

	union U_ISP_CORE_AWB_RED_OBJ_VALUE stRedObjValue;
	stRedObjValue.u32 = 0;
	stRedObjValue.bits.awb_red_obj_value = pstAwbCtl->awb_red_obj_value;
	isp_reg_writel(isp, stRedObjValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_OBJ_VALUE);

	union U_ISP_CORE_AWB_BLUE_OBJ_VALUE stBlueObjValue;
	stBlueObjValue.u32 = 0;
	stBlueObjValue.bits.awb_blue_obj_value = pstAwbCtl->awb_blue_obj_value;
	isp_reg_writel(isp, stBlueObjValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_OBJ_VALUE);

	return 0;
}

int Isp_Drv_F2k_Core_GetAwbSts(struct k510_isp_device *isp,ISP_CORE_AWB_STS_S *pAwbSts)
{

 	union U_ISP_CORE_AWB_BLUE_HIST_VALUE     gtAwbBHistVal;
  	gtAwbBHistVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_HIST_VALUE);
  	pAwbSts->awb_blue_hist_value = gtAwbBHistVal.u32;

  	union U_ISP_CORE_AWB_BLUE_HIST_PIXEL     gtAwbBHistPix;
  	gtAwbBHistPix.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_HIST_PIXEL);
  	pAwbSts->awb_blue_hist_pixel = gtAwbBHistPix.u32;

  	union U_ISP_CORE_AWB_RED_HIST_VALUE     gtAwbRHistVal;
  	gtAwbRHistVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_HIST_VALUE);
  	pAwbSts->awb_red_hist_value = gtAwbRHistVal.u32;

  	union U_ISP_CORE_AWB_RED_HIST_PIXEL      gtAwbRHistPix;
  	gtAwbRHistPix.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_HIST_VALUE );
  	pAwbSts->awb_red_hist_pixel = gtAwbRHistPix.u32;

  	union U_ISP_CORE_AWB_BYPASS_BLUE_HIST_VALUE   gtAwbBYPassBHistVal;
  	gtAwbBYPassBHistVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BYPASS_BLUE_HIST_VALUE);
  	pAwbSts->awb_bypass_blue_hist_value = gtAwbBYPassBHistVal.u32;

  	union U_ISP_CORE_AWB_BYPASS_BLUE_HIST_PIXEL   gtAwbBypassBHistPix;
  	gtAwbBypassBHistPix.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BYPASS_BLUE_HIST_PIXEL);
  	pAwbSts->awb_bypass_blue_hist_pixel = gtAwbBypassBHistPix.u32;

  	union U_ISP_CORE_AWB_BYPASS_RED_HIST_VALUE   gtAwbBYPASSRHistVal;
  	gtAwbBYPASSRHistVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BYPASS_RED_HIST_VALUE);
  	pAwbSts->awb_bypass_red_hist_value = gtAwbBYPASSRHistVal.u32;

  	union U_ISP_CORE_AWB_BYPASS_RED_HIST_PIXEL   gtAwbBypassRHistPix;
  	gtAwbBypassRHistPix.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BYPASS_RED_HIST_PIXEL);
  	pAwbSts->awb_bypass_red_hist_pixel = gtAwbBypassRHistPix.u32;

  	union U_ISP_CORE_AWB_RED_VALUE           gtAwbRVal;
  	gtAwbRVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_RED_VALUE);
  	pAwbSts->awb_red_value = gtAwbRVal.u32;

  	union U_ISP_CORE_AWB_GREEN_VALUE         gtAwbGVal;
  	gtAwbGVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_GREEN_VALUE);
  	pAwbSts->awb_green_value = gtAwbGVal.u32;

  	union U_ISP_CORE_AWB_BLUE_VALUE          gtAwbBVal;
  	gtAwbBVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_BLUE_VALUE);
  	pAwbSts->awb_blue_value = gtAwbBVal.u32;

  	union U_ISP_CORE_AWB_ORG_RED_VALUE       gtAwbORGRVal;
  	gtAwbORGRVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_ORG_RED_VALUE);
  	pAwbSts->awb_org_image_red_value = gtAwbORGRVal.u32;

  	union U_ISP_CORE_AWB_ORG_GREEN_VALUE     gtAwbORGGVal;
  	gtAwbORGGVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_ORG_GREEN_VALUE);
  	pAwbSts->awb_org_image_green_value = gtAwbORGGVal.u32;

  	union U_ISP_CORE_AWB_ORG_BLUE_VALUE      gtAwbORGBVal;
  	gtAwbORGBVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AWB_ORG_BLUE_VALUE);
  	pAwbSts->awb_org_image_blue_value = gtAwbORGBVal.u32;

  	return 0;
}
/*
*CSC
*/
int Isp_Drv_F2k_Core_SetCscCtl(struct k510_isp_device *isp,ISP_CORE_CSC_CTL_S *pstCscCtl)
{

	union U_ISP_CORE_CSC_R2Y_00 st00;
	st00.u32 = 0;
	st00.bits.csc_r2y_00 = pstCscCtl->csc_r2y_00;
	isp_reg_writel(isp,st00.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_00);

	union U_ISP_CORE_CSC_R2Y_01 st01;
	st01.u32 = 0;
	st01.bits.csc_r2y_01 = pstCscCtl->csc_r2y_01;
	isp_reg_writel(isp,st01.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_01);

	union U_ISP_CORE_CSC_R2Y_02 st02;
	st02.u32 = 0;
	st02.bits.csc_r2y_02 = pstCscCtl->csc_r2y_02;
    isp_reg_writel(isp,st02.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_02);

	union U_ISP_CORE_CSC_R2Y_10 st10;
	st10.u32 = 0;
	st10.bits.csc_r2y_10 = pstCscCtl->csc_r2y_10;
    isp_reg_writel(isp,st10.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_10);

	union U_ISP_CORE_CSC_R2Y_11 st11;
	st11.u32 = 0;
	st11.bits.csc_r2y_11 = pstCscCtl->csc_r2y_11;
	isp_reg_writel(isp,st11.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_11);

	union U_ISP_CORE_CSC_R2Y_12 st12;
	st12.u32 = 0;
	st12.bits.csc_r2y_12 = pstCscCtl->csc_r2y_12;
	isp_reg_writel(isp,st12.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_12);

	union U_ISP_CORE_CSC_R2Y_20 st20;
	st20.u32 = 0;
	st20.bits.csc_r2y_20 = pstCscCtl->csc_r2y_20;
	isp_reg_writel(isp,st20.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_20);

	union U_ISP_CORE_CSC_R2Y_21 st21;
	st21.u32 = 0;
	st21.bits.csc_r2y_21 = pstCscCtl->csc_r2y_21;
	isp_reg_writel(isp,st21.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_21);

	union U_ISP_CORE_CSC_R2Y_22 st22;
	st22.u32 = 0;
	st22.bits.csc_r2y_22 = pstCscCtl->csc_r2y_22;
	isp_reg_writel(isp,st22.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CSC_R2Y_22);
	return 0;
}
/*
* WDR
*/
int Isp_Drv_F2k_Core_SetWdrCtl(struct k510_isp_device *isp,ISP_CORE_WDR_CTL_S *pstWdrCtl)
{

	union U_ISP_CORE_WDR_CTL stWdrCtl;
	stWdrCtl.u32 = 0;
	stWdrCtl.bits.wdr_fusion_en = pstWdrCtl->wdr_fusion_en;
	stWdrCtl.bits.wdr_frame_sel = pstWdrCtl->wdr_frame_sel;
	stWdrCtl.bits.wdr_adp_adjust_en = pstWdrCtl->wdr_adp_adjust_en;
	stWdrCtl.bits.wdr_stab_en = pstWdrCtl->wdr_stab_en;
	stWdrCtl.bits.wdr_en = pstWdrCtl->wdr_en;
	stWdrCtl.bits.wdr_ghost_remove_en = pstWdrCtl->wdr_ghost_remove_en;
	stWdrCtl.bits.wdr_3frame_out_mode = pstWdrCtl->wdr_3frame_out_mode;
	stWdrCtl.bits.wdr_mode_sel = pstWdrCtl->wdr_mode_sel;
	stWdrCtl.bits.wdr_2frame_ex_ratio = pstWdrCtl->wdr_2frame_ex_ratio;
	stWdrCtl.bits.wdr_3frame_ex_ratio = pstWdrCtl->wdr_3frame_ex_ratio;
	stWdrCtl.bits.wdr_stat_img_sel = pstWdrCtl->wdr_stat_img_sel;
	stWdrCtl.bits.wdr_ltm_data_sel = pstWdrCtl->wdr_ltm_data_sel;
	stWdrCtl.bits.wdr_tz_data_sel = pstWdrCtl->wdr_tz_data_sel;
	stWdrCtl.bits.wdr_remove_purple_en = pstWdrCtl->wdr_remove_purple_en;
	isp_reg_writel(isp,stWdrCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_CTL);

	union U_ISP_CORE_WDR_OVER_EX_RATIO_TH1 stRatioTh1;
	stRatioTh1.u32 = 0;
	stRatioTh1.bits.wdr_over_ex_ratio_th1 = pstWdrCtl->wdr_over_ex_ratio_th1;
	isp_reg_writel(isp,stRatioTh1.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_OVER_EX_RATIO_TH1);

	union U_ISP_CORE_WDR_OVER_EX_RATIO_TH2 stRatioTh2;
	stRatioTh2.u32 = 0;
	stRatioTh2.bits.wdr_over_ex_ratio_th2 = pstWdrCtl->wdr_over_ex_ratio_th2;
	isp_reg_writel(isp,stRatioTh2.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_OVER_EX_RATIO_TH2);

	union U_ISP_CORE_WDR_FUSION_RATIO_TH stFusionRatio;
	stFusionRatio.u32 = 0;
	stFusionRatio.bits.wdr_fusion_ratio_th = pstWdrCtl->wdr_fusion_ratio_th;
	isp_reg_writel(isp,stFusionRatio.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_FUSION_RATIO_TH);

	union U_ISP_CORE_WDR_FUSION_VALUE1 stFusionValue1;
	stFusionValue1.u32 = 0;
	stFusionValue1.bits.wdr_fusion_value1 = pstWdrCtl->wdr_fusion_value1;
	isp_reg_writel(isp,stFusionValue1.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_FUSION_VALUE1);

	union U_ISP_CORE_WDR_FUSION_VALUE2 stFusionValue2;
	stFusionValue2.u32 = 0;
	stFusionValue2.bits.wdr_fusion_value2 = pstWdrCtl->wdr_fusion_value2;
	isp_reg_writel(isp,stFusionValue2.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_WDR_FUSION_VALUE2);
	return 0;
}
/*
*ADA
*/
int Isp_Drv_F2k_Core_SetAdaCtl(struct k510_isp_device *isp,ISP_CORE_ADA_CTL_S *pstAdaCtl)
{

 	union U_ISP_CORE_ADA_CTL stCtl;
	stCtl.u32 = 0;
	stCtl.bits.ada_rgb_gamma_en = pstAdaCtl->ada_rgb_gamma_en;
	stCtl.bits.ada_yuv_gamma_en = pstAdaCtl->ada_yuv_gamma_en;
	stCtl.bits.ada_adjust_en = pstAdaCtl->ada_adjust_en;
	stCtl.bits.ada_img_stab_en = pstAdaCtl->ada_img_stab_en;
	stCtl.bits.ada_ccr_en = pstAdaCtl->ada_ccr_en;
	stCtl.bits.ada_adp_en = pstAdaCtl->ada_adp_en;
	stCtl.bits.ada_adp_ccr_en = pstAdaCtl->ada_adp_ccr_en;
	stCtl.bits.ada_stat_mode_sel = pstAdaCtl->ada_stat_mode_sel;
	stCtl.bits.ada_enh_mode_sel = pstAdaCtl->ada_enh_mode_sel;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_CTL);

	union U_ISP_CORE_ADA_STAT_MAX_VALUE stStatMax;
	stStatMax.u32 = 0;
	stStatMax.bits.ada_stat_max_value = pstAdaCtl->ada_stat_max_value;
	isp_reg_writel(isp,stStatMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_STAT_MAX_VALUE);

	union U_ISP_CORE_ADA_AD_STREN_MAX_VALUE stStrenMax;
	stStrenMax.u32 = 0;
	stStrenMax.bits.ada_ad_stren_max_value = pstAdaCtl->ada_ad_stren_max_value;
	isp_reg_writel(isp,stStrenMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_AD_STREN_MAX_VALUE);

	union U_ISP_CORE_ADA_WIN_H_START stHStart;
	stHStart.u32 = 0;
	stHStart.bits.ada_win_h_start = pstAdaCtl->ada_win_h_start;
	isp_reg_writel(isp,stHStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_WIN_H_START);

	union U_ISP_CORE_ADA_WIN_V_START stVStart;
	stVStart.u32 = 0;
	stVStart.bits.ada_win_v_start = pstAdaCtl->ada_win_v_start;
	isp_reg_writel(isp,stVStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_WIN_V_START);

	union U_ISP_CORE_ADA_WIN_H_END stHEnd;
	stHEnd.u32 = 0;
	stHEnd.bits.ada_win_h_end = pstAdaCtl->ada_win_h_end;
	isp_reg_writel(isp,stHEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_WIN_H_END);

	union U_ISP_CORE_ADA_WIN_V_END stVEnd;
	stVEnd.u32 = 0;
	stVEnd.bits.ada_win_v_end = pstAdaCtl->ada_win_v_end;
	isp_reg_writel(isp,stVEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ADA_WIN_V_END);
	return 0;
}
/*
*rgbir
*/
int Isp_Drv_F2k_Core_SetRgbIrCtl(struct k510_isp_device *isp,ISP_CORE_RGBIR_CTL_S *pstRgbIrCtl)
{

	union U_ISP_CORE_RGBIR_CTL stCtl;
	stCtl.u32 = 0;
	stCtl.bits.rgbir_en = pstRgbIrCtl->rgbir_en;
	stCtl.bits.rgbir_rtf_en = pstRgbIrCtl->rgbir_rtf_en;
	stCtl.bits.rgbir_rpc_en = pstRgbIrCtl->rgbir_rpc_en;
	stCtl.bits.rgbir_fusion_en = pstRgbIrCtl->rgbir_fusion_en;
	stCtl.bits.rgbir_output_sel = pstRgbIrCtl->rgbir_output_sel;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_CTL);

	union U_ISP_CORE_RGBIR_RPC_MAX_VALUE stMax;
	stMax.u32 = 0;
	stMax.bits.rgbir_rpc_max_value = pstRgbIrCtl->rgbir_rpc_max_value;
	isp_reg_writel(isp,stMax.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_RPC_MAX_VALUE);

	union U_ISP_CORE_RGBIR_RPC_COLOR_COFF stColorCoff;
	stColorCoff.u32 = 0;
	stColorCoff.bits.rgbir_rpc_color_coff = pstRgbIrCtl->rgbir_rpc_color_coff;
	isp_reg_writel(isp,stColorCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_RPC_COLOR_COFF);

	union U_ISP_CORE_RGBIR_RPC_LUMA_COFF stLumaCoff;
	stLumaCoff.u32 = 0;
	stLumaCoff.bits.rgbir_rpc_luma_coff = pstRgbIrCtl->rgbir_rpc_luma_coff;
	isp_reg_writel(isp,stLumaCoff.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_RPC_LUMA_COFF);

	union U_ISP_CORE_RGBIR_RPC_TH stTh;
	stTh.u32 = 0;
	stTh.bits.rgbir_rpc_th = pstRgbIrCtl->rgbir_rpc_th;
	isp_reg_writel(isp,stTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_RPC_TH);

	union U_ISP_CORE_RGBIR_RPC_TH1 stTh1;
	stTh1.u32 = 0;
	stTh1.bits.rgbir_rpc_th1 = pstRgbIrCtl->rgbir_rpc_th1;
	isp_reg_writel(isp,stTh1.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_RGBIR_RPC_TH1);

	return 0;
}
/*
* 2D Noise Reduction
*/
int Isp_Drv_F2k_Core_Set2DnrCtl(struct k510_isp_device *isp,ISP_CORE_2DNR_CTL_S *pst2DnrCtl)
{

	union U_ISP_CORE_2DNR_CTL stCtl;
	stCtl.u32 = 0;
	stCtl.bits.core_2dnr_pcf_en = pst2DnrCtl->core_2dnr_pcf_en;
	stCtl.bits.core_2dnr_raw_en = pst2DnrCtl->core_2dnr_raw_en;
	stCtl.bits.core_2dnr_edge_en = pst2DnrCtl->core_2dnr_edge_en;
	stCtl.bits.core_2dnr_bap_en = pst2DnrCtl->core_2dnr_bap_en;
	stCtl.bits.core_2dnr_luma_en = pst2DnrCtl->core_2dnr_luma_en;
	stCtl.bits.core_2dnr_chroma_en = pst2DnrCtl->core_2dnr_chroma_en;
	stCtl.bits.core_2dnr_pcf_adp_en = pst2DnrCtl->core_2dnr_pcf_adp_en;
	stCtl.bits.core_2dnr_raw_adp_en = pst2DnrCtl->core_2dnr_raw_adp_en;
	stCtl.bits.core_2dnr_luma_adp_en = pst2DnrCtl->core_2dnr_luma_adp_en;
	stCtl.bits.core_2dnr_chroma_adp_en = pst2DnrCtl->core_2dnr_chroma_adp_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_CTL);

	union U_ISP_CORE_2DNR_RAW_INTENSITY stRawInte;
	stRawInte.u32 = 0;
	stRawInte.bits.core_2dnr_raw_intensity = pst2DnrCtl->core_2dnr_raw_intensity;
	isp_reg_writel(isp,stRawInte.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_RAW_INTENSITY);

	union U_ISP_CORE_2DNR_BAP_INTENSITY stBapInte;
	stBapInte.u32 = 0;
	stBapInte.bits.core_2dnr_bap_intensity = pst2DnrCtl->core_2dnr_bap_intensity;
	isp_reg_writel(isp,stBapInte.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_BAP_INTENSITY);

	union U_ISP_CORE_2DNR_EDGE_INTENSITY stEdgeInte;
	stEdgeInte.u32 = 0;
	stEdgeInte.bits.core_2dnr_edge_intensity = pst2DnrCtl->core_2dnr_edge_intensity;
	isp_reg_writel(isp,stEdgeInte.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_EDGE_INTENSITY);

	union U_ISP_CORE_2DNR_LUMA_INTENSITY stLumaInte;
	stLumaInte.u32 = 0;
	stLumaInte.bits.core_2dnr_luma_intensity = pst2DnrCtl->core_2dnr_luma_intensity;
	isp_reg_writel(isp,stLumaInte.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_LUMA_INTENSITY);

	union U_ISP_CORE_2DNR_CHROMA_INTENSITY stChromaInte;
	stChromaInte.u32 = 0;
	stChromaInte.bits.core_2dnr_chroma_intensity = pst2DnrCtl->core_2dnr_chroma_intensity;
	isp_reg_writel(isp,stChromaInte.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_2DNR_CHROMA_INTENSITY);

	return 0;
}
/*
* 3D Noise Reduction
*/
int Isp_Drv_F2k_Core_Set3DnrCtl(struct k510_isp_device *isp,ISP_CORE_3DNR_CTL_S *pst3DnrCtl)
{

 	union U_ISP_CORE_3DNR_CTL stCtl;
	stCtl.u32 = 0;
	stCtl.bits.core_3dnr_en = pst3DnrCtl->core_3dnr_en;
	stCtl.bits.core_3dnr_pre_luma_en = pst3DnrCtl->core_3dnr_pre_luma_en;
	stCtl.bits.core_3dnr_pre_chroma_en = pst3DnrCtl->core_3dnr_pre_chroma_en;
	stCtl.bits.core_3dnr_main_luma_en = pst3DnrCtl->core_3dnr_main_luma_en;
	stCtl.bits.core_3dnr_main_chroma_en = pst3DnrCtl->core_3dnr_main_chroma_en;
	stCtl.bits.core_3dnr_post_luma_en = pst3DnrCtl->core_3dnr_post_luma_en;
	stCtl.bits.core_3dnr_post_chroma_en = pst3DnrCtl->core_3dnr_post_chroma_en;
	stCtl.bits.core_3dnr_2d_luma_en = pst3DnrCtl->core_3dnr_2d_luma_en;
	stCtl.bits.core_3dnr_2d_chroma_en = pst3DnrCtl->core_3dnr_2d_chroma_en;
	stCtl.bits.core_3dnr_wb_en = pst3DnrCtl->core_3dnr_wb_en;
	stCtl.bits.core_3dnr_wb_sel = pst3DnrCtl->core_3dnr_wb_sel;
	stCtl.bits.core_3dnr_adp_luma_en = pst3DnrCtl->core_3dnr_adp_luma_en;
	stCtl.bits.core_3dnr_adp_chroma_en = pst3DnrCtl->core_3dnr_adp_chroma_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_CTL);

	union U_ISP_CORE_3DNR_PRE_LUMA_TH stPreLumaTh;
	stPreLumaTh.u32 = 0;
	stPreLumaTh.bits.core_3dnr_pre_luma_th = pst3DnrCtl->core_3dnr_pre_luma_th;
	isp_reg_writel(isp,stPreLumaTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_PRE_LUMA_TH);

	union U_ISP_CORE_3DNR_PRE_LUMA_INTENSITY stPreLumaInten;
	stPreLumaInten.u32 = 0;
	stPreLumaInten.bits.core_3dnr_pre_luma_intensity = pst3DnrCtl->core_3dnr_pre_luma_intensity;
	isp_reg_writel(isp,stPreLumaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_PRE_LUMA_INTENSITY);

	union U_ISP_CORE_3DNR_PRE_CHROMA_INTENSITY stPreChromaInten;
	stPreChromaInten.u32 = 0;
	stPreChromaInten.bits.core_3dnr_pre_chroma_intensity = pst3DnrCtl->core_3dnr_pre_chroma_intensity;
    isp_reg_writel(isp,stPreChromaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_PRE_CHROMA_INTENSITY);

	union U_ISP_CORE_3DNR_MID_FILTER_TH stMidFilterTh;
	stMidFilterTh.u32 = 0;
	stMidFilterTh.bits.core_3dnr_mid_filter_th = pst3DnrCtl->core_3dnr_mid_filter_th;
	isp_reg_writel(isp,stMidFilterTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_MID_FILTER_TH);

	union U_ISP_CORE_3DNR_PRE_MID_FILTER_TH stPreMidFilterTh;
	stPreMidFilterTh.u32 = 0;
	stPreMidFilterTh.bits.core_3dnr_pre_mid_filter_th = pst3DnrCtl->core_3dnr_pre_mid_filter_th;
	isp_reg_writel(isp,stPreMidFilterTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_PRE_MID_FILTER_TH);

	union U_ISP_CORE_3DNR_CUR_FILTER_TH stCurFilterTh;
	stCurFilterTh.u32 = 0;
	stCurFilterTh.bits.core_3dnr_cur_mid_filter_th = pst3DnrCtl->core_3dnr_cur_mid_filter_th;
	isp_reg_writel(isp,stCurFilterTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_CUR_FILTER_TH);

	union U_ISP_CORE_3DNR_LOW_PASS_FILTER_TH stLowFilterTh;
	stLowFilterTh.u32 = 0;
	stLowFilterTh.bits.core_3dnr_low_pass_filter_th = pst3DnrCtl->core_3dnr_low_pass_filter_th;
	isp_reg_writel(isp,stLowFilterTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_LOW_PASS_FILTER_TH);

	union U_ISP_CORE_3DNR_LUMA_TH stLumaTh;
	stLumaTh.u32 = 0;
	stLumaTh.bits.core_3dnr_luma_th = pst3DnrCtl->core_3dnr_luma_th;
	isp_reg_writel(isp,stLumaTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_LUMA_TH);

	union U_ISP_CORE_3DNR_MIN_VALUE stMinValue;
	stMinValue.u32 = 0;
	stMinValue.bits.core_3dnr_min_value = pst3DnrCtl->core_3dnr_min_value;
    isp_reg_writel(isp,stMinValue.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_MIN_VALUE);

	union U_ISP_CORE_3DNR_LUMA_INTENSITY stLumaInten;
	stLumaInten.u32 = 0;
	stLumaInten.bits.core_3dnr_luma_intensity = pst3DnrCtl->core_3dnr_luma_intensity;
	isp_reg_writel(isp,stLumaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_LUMA_INTENSITY);

	union U_ISP_CORE_3DNR_CHROMA_INTENSITY stChromaInten;
	stChromaInten.u32 = 0;
	stChromaInten.bits.core_3dnr_chroma_intensity = pst3DnrCtl->core_3dnr_chroma_intensity;
	isp_reg_writel(isp,stChromaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_CHROMA_INTENSITY);

	union U_ISP_CORE_3DNR_POST_EDGE_TH stPostEdge;
	stPostEdge.u32 = 0;
	stPostEdge.bits.core_3dnr_post_edge_th = pst3DnrCtl->core_3dnr_post_edge_th;
	isp_reg_writel(isp,stPostEdge.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_POST_EDGE_TH);

	union U_ISP_CORE_3DNR_POST_LUMA_INTENSITY stPostLumaInten;
	stPostLumaInten.u32 = 0;
	stPostLumaInten.bits.core_3dnr_post_luma_intensity = pst3DnrCtl->core_3dnr_post_luma_intensity;
	isp_reg_writel(isp,stPostLumaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_POST_LUMA_INTENSITY);

	union U_ISP_CORE_3DNR_POST_CHROMA_INTENSITY stPostChromaInten;
	stPostChromaInten.u32 = 0;
	stPostChromaInten.bits.core_3dnr_post_chroma_intensity = pst3DnrCtl->core_3dnr_post_chroma_intensity;
	isp_reg_writel(isp,stPostChromaInten.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_3DNR_POST_CHROMA_INTENSITY);

	return 0;
}
/*
*ENH Control
*/
int Isp_Drv_F2k_Core_SetEnhLtmCtl(struct k510_isp_device *isp,ISP_CORE_ENH_CTL_S *pstEnhCtl)
{

	union U_ISP_CORE_ENH_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);
	stCtl.bits.enh_ltm_en = pstEnhCtl->enh_ltm_en;
	stCtl.bits.enh_adp_ltm_en = pstEnhCtl->enh_adp_ltm_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);

	union U_ISP_CORE_LTM_GAIN stLtmGain;
	stLtmGain.u32 = 0;
	stLtmGain.bits.ltm_gain = pstEnhCtl->ltm_gain;
	isp_reg_writel(isp,stLtmGain.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LTM_GAIN);

	union U_ISP_CORE_LTM_TH stLtmTh;
	stLtmTh.u32 = 0;
	stLtmTh.bits.ltm_th = pstEnhCtl->ltm_th;
	isp_reg_writel(isp,stLtmTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LTM_TH);
	return 0;
}

int Isp_Drv_F2k_Core_SetEnhCCCtl(struct k510_isp_device *isp,ISP_CORE_ENH_CTL_S *pstEnhCtl)
{


	union U_ISP_CORE_ENH_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);
	stCtl.bits.enh_cc_en = pstEnhCtl->enh_cc_en;
	stCtl.bits.enh_adp_cc_en = pstEnhCtl->enh_adp_cc_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);

	union U_ISP_CORE_ENH_NR_TH stNrTh;
	stNrTh.u32 = 0;
	stNrTh.bits.enh_nr_th = pstEnhCtl->enh_nr_th;
	isp_reg_writel(isp,stNrTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_NR_TH);

	union U_ISP_CORE_ENH_TH1 stTh1;
	stTh1.u32 = 0;
	stTh1.bits.enh_th1 = pstEnhCtl->enh_th1;
	isp_reg_writel(isp,stTh1.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_TH1);

	union U_ISP_CORE_ENH_TH2 stTh2;
	stTh2.u32 = 0;
	stTh2.bits.enh_th2 = pstEnhCtl->enh_th2;
	isp_reg_writel(isp,stTh2.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_TH2);
	return 0;
}

int Isp_Drv_F2k_Core_SetEnhSharpenCtl(struct k510_isp_device *isp,ISP_CORE_ENH_CTL_S *pstEnhCtl)
{


	union U_ISP_CORE_ENH_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);
	stCtl.bits.enh_sharp_en = pstEnhCtl->enh_sharp_en;
	stCtl.bits.enh_adp_sharp_en = pstEnhCtl->enh_adp_sharp_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_ENH_CTL);

	union U_ISP_CORE_SHARP_GAIN stSharpgain;
	stSharpgain.u32 = 0;
	stSharpgain.bits.sharp_gain = pstEnhCtl->sharp_gain;
	isp_reg_writel(isp,stSharpgain.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_SHARP_GAIN);
	return 0;
}
/*
* POST Control
*/
int Isp_Drv_F2k_Core_SetPostContCtl(struct k510_isp_device *isp,ISP_CORE_POST_CTL_S *pstPostCtl)
{


	union U_ISP_CORE_POST_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);
	stCtl.bits.post_cont_ad_en = pstPostCtl->post_cont_ad_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);

	union U_ISP_CORE_CONT_GAIN stContGain;
	stContGain.u32 = 0;
	stContGain.bits.cont_ad_intensity = pstPostCtl->cont_ad_intensity;
	isp_reg_writel(isp,stContGain.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_CONT_GAIN);
	return 0;
}

int Isp_Drv_F2k_Core_SetPostLumaCtl(struct k510_isp_device *isp,ISP_CORE_POST_CTL_S *pstPostCtl)
{

	union U_ISP_CORE_POST_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);
	stCtl.bits.post_luma_ad_en = pstPostCtl->post_luma_ad_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);

	union U_ISP_CORE_LUMA_GAIN stLumaTh;
	stLumaTh.u32 = 0;
	stLumaTh.bits.luma_ad_intensity = pstPostCtl->luma_ad_intensity;
	isp_reg_writel(isp,stLumaTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LUMA_GAIN);
	return 0;
}

int Isp_Drv_F2k_Core_SetPostSatuCtl(struct k510_isp_device *isp,ISP_CORE_POST_CTL_S *pstPostCtl)
{

	union U_ISP_CORE_POST_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);
	stCtl.bits.post_satu_ad_en = pstPostCtl->post_satu_ad_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);

	union U_ISP_CORE_SATU_GAIN stSatuTh;
	stSatuTh.u32 = 0;
	stSatuTh.bits.satu_ad_intensity = pstPostCtl->satu_ad_intensity;
	isp_reg_writel(isp,stSatuTh.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_SATU_GAIN);
	return 0;
}
/*
* OTC
*/
int Isp_Drv_F2k_Core_SetOtcCtl(struct k510_isp_device *isp,ISP_CORE_OTC_CTL_S *pstOtcCtl)
{

	union U_ISP_CORE_POST_CTL stCtl;
	stCtl.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);
	stCtl.bits.post_otc_en = pstOtcCtl->post_otc_en;
	stCtl.bits.otc_yc_sel = pstOtcCtl->otc_yc_sel;
	stCtl.bits.otc_uv_format_sel = pstOtcCtl->otc_uv_format_sel;
	stCtl.bits.otc_hsync_pol_sel = pstOtcCtl->otc_hsync_pol_sel;
	stCtl.bits.otc_vsync_pol_sel = pstOtcCtl->otc_vsync_pol_sel;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_POST_CTL);

	union U_ISP_CORE_OTC_STT_VR stOtcVr;
  	stOtcVr.u32 = 0;
  	stOtcVr.bits.otc_stt_vr = pstOtcCtl->otc_stt_vr;
  	isp_reg_writel(isp,stOtcVr.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_OTC_STT_VR);

  	union U_ISP_CORE_OTC_STT_HR stOtcHr;
  	stOtcHr.u32 = 0;
  	stOtcHr.bits.otc_stt_hr = pstOtcCtl->otc_stt_hr;
  	isp_reg_writel(isp,stOtcHr.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_OTC_STT_HR);

  	union U_ISP_CORE_OTC_HEIGHT stOtcHeight;
  	stOtcHeight.u32 = 0;
  	stOtcHeight.bits.otc_height = pstOtcCtl->otc_height;
  	isp_reg_writel(isp,stOtcHeight.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_OTC_HEIGHT);

  	union U_ISP_CORE_OTC_WIDTH  stOtcWidth;
  	stOtcWidth.u32 = 0;
  	stOtcWidth.bits.otc_width = pstOtcCtl->otc_width;
  	isp_reg_writel(isp,stOtcWidth.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_OTC_WIDTH);
	return 0;
}
/*
*LDC
*/
int Isp_Drv_F2k_Core_SetLdcCtl(struct k510_isp_device *isp,ISP_CORE_LDC_CTL_S *pstLdcCtl)
{

	union U_ISP_CORE_LDC_CTL stCtl;
	stCtl.u32 = 0;
	stCtl.bits.ldc_en = pstLdcCtl->ldc_en;
	stCtl.bits.ldc_arith_en = pstLdcCtl->ldc_arith_en;
	isp_reg_writel(isp,stCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_CTL);

	union U_ISP_CORE_LDC_REQ_FREQ stReqFreq;
	stReqFreq.u32 = 0;
	stReqFreq.bits.ldc_req_freq = pstLdcCtl->ldc_req_freq;
	stReqFreq.bits.ldc_stt_ln = pstLdcCtl->ldc_stt_ln;
	isp_reg_writel(isp,stReqFreq.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_REQ_FREQ);

	union U_ISP_CORE_LDC_H_CENTER_POS stHCenterPos;
	stHCenterPos.u32 = 0;
	stHCenterPos.bits.ldc_h_center_pos = pstLdcCtl->ldc_h_center_pos;
	isp_reg_writel(isp,stHCenterPos.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_H_CENTER_POS);

	union U_ISP_CORE_LDC_V_CENTER_POS stVCenterPos;
	stVCenterPos.u32 = 0;
	stVCenterPos.bits.ldc_v_center_pos = pstLdcCtl->ldc_v_center_pos;
	isp_reg_writel(isp,stVCenterPos.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_V_CENTER_POS);

	union U_ISP_CORE_LDC_RECTIFY_CR stRecifyCr;
	stRecifyCr.u32 = 0;
	stRecifyCr.bits.ldc_rectify_cr = pstLdcCtl->ldc_rectify_cr;
	isp_reg_writel(isp,stRecifyCr.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_RECTIFY_CR);

	union U_ISP_CORE_LDC_RECTIFY_CZ stRecifyCz;
	stRecifyCz.u32 = 0;
	stRecifyCz.bits.ldc_rectify_cz = pstLdcCtl->ldc_rectify_cz;
	isp_reg_writel(isp,stRecifyCz.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_LDC_RECTIFY_CZ);
	return 0;
}
/*
* AF Statistics
*/
int Isp_Drv_F2k_Core_SetAfCtl(struct k510_isp_device *isp,ISP_CORE_AF_CTL_S *pstAfCtl)
{

	union U_ISP_CORE_AF_CTL stAfCtl;
	stAfCtl.u32 = 0;
	stAfCtl.bits.af_stat_en = pstAfCtl->af_stat_en;
	stAfCtl.bits.af_stat_mode_sel = pstAfCtl->af_stat_mode_sel;
	isp_reg_writel(isp,stAfCtl.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_CTL);

	union U_ISP_CORE_AF_STAT_WIN_H_START stAfWinHStart;
	stAfWinHStart.u32 = 0;
	stAfWinHStart.bits.af_stat_win_h_start = pstAfCtl->af_stat_win_h_start;
	isp_reg_writel(isp,stAfWinHStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_STAT_WIN_H_START);

	union U_ISP_CORE_AF_STAT_WIN_V_START stAfWinVStart;
	stAfWinVStart.u32 = 0;
	stAfWinVStart.bits.af_stat_win_v_start = pstAfCtl->af_stat_win_v_start;
	isp_reg_writel(isp,stAfWinVStart.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_STAT_WIN_V_START);

	union U_ISP_CORE_AF_STAT_WIN_H_END stAfWinHEnd;
	stAfWinHEnd.u32 = 0;
	stAfWinHEnd.bits.af_stat_win_h_end = pstAfCtl->af_stat_win_h_end;
	isp_reg_writel(isp,stAfWinHEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_STAT_WIN_H_END);

	union U_ISP_CORE_AF_STAT_WIN_V_END stAfWinVEnd;
	stAfWinVEnd.u32 = 0;
	stAfWinVEnd.bits.af_stat_win_v_end = pstAfCtl->af_stat_win_v_end;
	isp_reg_writel(isp,stAfWinVEnd.u32,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_STAT_WIN_V_END);

	return 0;
}

int Isp_Drv_F2k_Core_GetAfSts(struct k510_isp_device *isp,ISP_CORE_AF_STS_S *pgtAfSts)
{


	union U_ISP_CORE_AF_MID_FRQ_DATA         gtAfMidFrqData;
	gtAfMidFrqData.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_MID_FRQ_DATA);
	pgtAfSts->af_mid_frq_data = gtAfMidFrqData.u32;

	union U_ISP_CORE_AF_HIGH_FRQ_DATA        gtAfHighFrqData;
	gtAfHighFrqData.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_HIGH_FRQ_DATA);
	pgtAfSts->af_high_frq_data = gtAfHighFrqData.u32;

	union U_ISP_CORE_AF_STAT_PIXEL_NUM       gtAfStatPixelNum;
	gtAfStatPixelNum.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_AF_STAT_PIXEL_NUM);
	pgtAfSts->af_stat_pixel_num = gtAfStatPixelNum.u32;
	return 0;
}

int Isp_Drv_F2k_Core_SetRamWrStatus(struct k510_isp_device *isp,unsigned int data)
{


	isp_reg_writel(isp,data,ISP_IOMEM_F2K_CORE,ISP_CORE_RAM_WR_STATUS);

	return 0;
}

int Isp_Drv_F2k_Core_GetRamRdStatus(struct k510_isp_device *isp)
{


	int isp_ram_ready = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,ISP_CORE_RAM_RD_STATUS);

	return isp_ram_ready;
}

/*
*F2K Reg Opt
*/

int Isp_Drv_F2k_Core_SetReg(struct k510_isp_device *isp,ISP_CORE_REG_VAL_S *pRegVal)
{
	union U_ISP_CORE_REG_CFG_ADDR stRegAddr;
	union U_ISP_CORE_REG_CFG_VALUE stRegVal;
	stRegAddr.u32 = pRegVal->reg_addr;
	stRegVal.u32 = pRegVal->reg_value;
	isp_reg_writel(isp,stRegVal.u32,ISP_IOMEM_F2K_CORE,stRegAddr.u32);
	return 0;
}

int Isp_Drv_F2k_Core_GetReg(struct k510_isp_device *isp,ISP_CORE_REG_VAL_S *pRegVal)
{
	union U_ISP_CORE_REG_CFG_ADDR gtRegAddr;
	union U_ISP_CORE_REG_CFG_VALUE gtRegVal;
	gtRegAddr.u32 = pRegVal->reg_addr;
	gtRegVal.u32 = isp_reg_readl(isp,ISP_IOMEM_F2K_CORE,gtRegAddr.u32);
	pRegVal->reg_value = gtRegVal.u32;
	return 0;
}