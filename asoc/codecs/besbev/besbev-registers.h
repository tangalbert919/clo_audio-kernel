/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _BESBEV_REGISTERS_H
#define _BESBEV_REGISTERS_H

#define BESBEV_BASE          0x3000
#define BESBEV_ANA_BASE_ADDR 0x3000
#define BESBEV_DIG_BASE_ADDR 0x3400

#define BESBEV_REG(reg)  ((reg > BESBEV_DIG_BASE_ADDR) ? \
			   (reg - BESBEV_DIG_BASE_ADDR) : \
			   (reg - BESBEV_ANA_BASE_ADDR))

enum {
	REG_NO_ACCESS,
	RD_REG,
	WR_REG,
	RD_WR_REG
};

#define BESBEV_ANA_MICBIAS_MICB_1_2_EN		(BESBEV_ANA_BASE_ADDR+0x040)
#define BESBEV_ANA_MICBIAS_MICB_3_EN		(BESBEV_ANA_BASE_ADDR+0x041)
#define BESBEV_ANA_MICBIAS_LDO_1_SETTING	(BESBEV_ANA_BASE_ADDR+0x042)
#define BESBEV_ANA_MICBIAS_LDO_1_CTRL		(BESBEV_ANA_BASE_ADDR+0x043)
#define BESBEV_ANA_TX_AMIC1			(BESBEV_ANA_BASE_ADDR+0x047)
#define BESBEV_ANA_TX_AMIC2			(BESBEV_ANA_BASE_ADDR+0x048)
#define BESBEV_ANA_NCP_EN			(BESBEV_ANA_BASE_ADDR+0x077)
#define BESBEV_ANA_NCP_VCTRL			(BESBEV_ANA_BASE_ADDR+0x07C)
#define BESBEV_ANA_HPHPA_CNP_CTL_1		(BESBEV_ANA_BASE_ADDR+0x083)
#define BESBEV_ANA_HPHPA_CNP_CTL_2		(BESBEV_ANA_BASE_ADDR+0x084)
#define BESBEV_ANA_HPHPA_PA_STATUS		(BESBEV_ANA_BASE_ADDR+0x087)
#define BESBEV_ANA_HPHPA_FSM_CLK		(BESBEV_ANA_BASE_ADDR+0x088)
#define BESBEV_ANA_HPHPA_L_GAIN			(BESBEV_ANA_BASE_ADDR+0x08B)
#define BESBEV_ANA_HPHPA_R_GAIN			(BESBEV_ANA_BASE_ADDR+0x08C)
#define BESBEV_ANA_HPHPA_SPARE_CTL		(BESBEV_ANA_BASE_ADDR+0x08E)
#define BESBEV_SWR_HPHPA_HD2			(BESBEV_ANA_BASE_ADDR+0x090)
#define BESBEV_ANA_SURGE_EN			(BESBEV_ANA_BASE_ADDR+0x097)
#define BESBEV_ANA_COMBOPA_CTL			(BESBEV_ANA_BASE_ADDR+0x09B)
#define BESBEV_ANA_COMBOPA_CTL_4		(BESBEV_ANA_BASE_ADDR+0x09F)
#define BESBEV_ANA_COMBOPA_CTL_5		(BESBEV_ANA_BASE_ADDR+0x0A0)
#define BESBEV_ANA_RXLDO_CTL			(BESBEV_ANA_BASE_ADDR+0x0B2)
#define BESBEV_ANA_MBIAS_EN			(BESBEV_ANA_BASE_ADDR+0x0B4)

#define BESBEV_DIG_SWR_CHIP_ID0			(BESBEV_DIG_BASE_ADDR+0x001)
#define BESBEV_DIG_SWR_CHIP_ID1			(BESBEV_DIG_BASE_ADDR+0x002)
#define BESBEV_DIG_SWR_CHIP_ID2			(BESBEV_DIG_BASE_ADDR+0x003)
#define BESBEV_DIG_SWR_CHIP_ID3			(BESBEV_DIG_BASE_ADDR+0x004)
#define BESBEV_DIG_SWR_SWR_TX_CLK_RATE		(BESBEV_DIG_BASE_ADDR+0x040)
#define BESBEV_DIG_SWR_CDC_RST_CTL		(BESBEV_DIG_BASE_ADDR+0x041)
#define BESBEV_DIG_SWR_TOP_CLK_CFG		(BESBEV_DIG_BASE_ADDR+0x042)
#define BESBEV_DIG_SWR_CDC_RX_CLK_CTL		(BESBEV_DIG_BASE_ADDR+0x043)
#define BESBEV_DIG_SWR_CDC_TX_CLK_CTL		(BESBEV_DIG_BASE_ADDR+0x044)
#define BESBEV_DIG_SWR_SWR_RST_EN		(BESBEV_DIG_BASE_ADDR+0x045)
#define BESBEV_DIG_SWR_CDC_RX_RST		(BESBEV_DIG_BASE_ADDR+0x047)
#define BESBEV_DIG_SWR_CDC_RX0_CTL		(BESBEV_DIG_BASE_ADDR+0x048)
#define BESBEV_DIG_SWR_CDC_RX1_CTL		(BESBEV_DIG_BASE_ADDR+0x049)
#define BESBEV_DIG_SWR_CDC_TX_ANA_MODE_0_1	(BESBEV_DIG_BASE_ADDR+0x04B)
#define BESBEV_DIG_SWR_CDC_COMP_CTL_0		(BESBEV_DIG_BASE_ADDR+0x04F)
#define BESBEV_DIG_SWR_CDC_RX_DELAY_CTL		(BESBEV_DIG_BASE_ADDR+0x052)
#define BESBEV_DIG_SWR_CDC_RX_GAIN_0		(BESBEV_DIG_BASE_ADDR+0x053)
#define BESBEV_DIG_SWR_CDC_RX_GAIN_1		(BESBEV_DIG_BASE_ADDR+0x054)
#define BESBEV_DIG_SWR_CDC_RX_GAIN_CTL		(BESBEV_DIG_BASE_ADDR+0x057)
#define BESBEV_DIG_SWR_CDC_TX0_CTL		(BESBEV_DIG_BASE_ADDR+0x060)
#define BESBEV_DIG_SWR_CDC_TX1_CTL		(BESBEV_DIG_BASE_ADDR+0x061)
#define BESBEV_DIG_SWR_CDC_TX_RST		(BESBEV_DIG_BASE_ADDR+0x063)
#define BESBEV_DIG_SWR_CDC_REQ0_CTL		(BESBEV_DIG_BASE_ADDR+0x064)
#define BESBEV_DIG_SWR_CDC_REQ1_CTL		(BESBEV_DIG_BASE_ADDR+0x065)
#define BESBEV_DIG_SWR_CDC_RST			(BESBEV_DIG_BASE_ADDR+0x067)
#define BESBEV_DIG_SWR_CDC_AMIC_CTL		(BESBEV_DIG_BASE_ADDR+0x06A)
#define BESBEV_DIG_SWR_CDC_DMIC_CTL		(BESBEV_DIG_BASE_ADDR+0x06B)
#define BESBEV_DIG_SWR_CDC_DMIC1_CTL		(BESBEV_DIG_BASE_ADDR+0x06C)
#define BESBEV_DIG_SWR_CDC_DMIC1_RATE		(BESBEV_DIG_BASE_ADDR+0x06D)
#define BESBEV_DIG_SWR_PDM_WD_CTL0		(BESBEV_DIG_BASE_ADDR+0x070)
#define BESBEV_DIG_SWR_PDM_WD_CTL1		(BESBEV_DIG_BASE_ADDR+0x071)
#define BESBEV_DIG_SWR_INTR_MODE		(BESBEV_DIG_BASE_ADDR+0x080)
#define BESBEV_DIG_SWR_INTR_MASK_0		(BESBEV_DIG_BASE_ADDR+0x081)
#define BESBEV_DIG_SWR_INTR_MASK_1		(BESBEV_DIG_BASE_ADDR+0x082)
#define BESBEV_DIG_SWR_INTR_MASK_2		(BESBEV_DIG_BASE_ADDR+0x083)
#define BESBEV_DIG_SWR_INTR_STATUS_0		(BESBEV_DIG_BASE_ADDR+0x084)
#define BESBEV_DIG_SWR_INTR_STATUS_1		(BESBEV_DIG_BASE_ADDR+0x085)
#define BESBEV_DIG_SWR_INTR_STATUS_2		(BESBEV_DIG_BASE_ADDR+0x086)
#define BESBEV_DIG_SWR_INTR_CLEAR_0		(BESBEV_DIG_BASE_ADDR+0x087)
#define BESBEV_DIG_SWR_INTR_CLEAR_1		(BESBEV_DIG_BASE_ADDR+0x088)
#define BESBEV_DIG_SWR_INTR_CLEAR_2		(BESBEV_DIG_BASE_ADDR+0x089)
#define BESBEV_DIG_SWR_INTR_LEVEL_0		(BESBEV_DIG_BASE_ADDR+0x08A)
#define BESBEV_DIG_SWR_INTR_LEVEL_1		(BESBEV_DIG_BASE_ADDR+0x08B)
#define BESBEV_DIG_SWR_INTR_LEVEL_2		(BESBEV_DIG_BASE_ADDR+0x08C)
#define BESBEV_DIG_SWR_CDC_CONN_RX0_CTL		(BESBEV_DIG_BASE_ADDR+0x093)
#define BESBEV_DIG_SWR_CDC_CONN_RX1_CTL		(BESBEV_DIG_BASE_ADDR+0x094)
#define BESBEV_DIG_SWR_LOOP_BACK_MODE		(BESBEV_DIG_BASE_ADDR+0x097)
#define BESBEV_DIG_SWR_DRIVE_STRENGTH_0		(BESBEV_DIG_BASE_ADDR+0x0A0)
#define BESBEV_DIG_SWR_DIG_DEBUG_CTL		(BESBEV_DIG_BASE_ADDR+0x0AB)
#define BESBEV_DIG_SWR_DIG_DEBUG_EN		(BESBEV_DIG_BASE_ADDR+0x0AC)
#define BESBEV_DIG_SWR_DEM_BYPASS_DATA0		(BESBEV_DIG_BASE_ADDR+0x0B0)
#define BESBEV_DIG_SWR_DEM_BYPASS_DATA1		(BESBEV_DIG_BASE_ADDR+0x0B1)
#define BESBEV_DIG_SWR_DEM_BYPASS_DATA2		(BESBEV_DIG_BASE_ADDR+0x0B2)
#define BESBEV_DIG_SWR_DEM_BYPASS_DATA3		(BESBEV_DIG_BASE_ADDR+0x0B3)

#define BESBEV_ANALOG_REGISTERS_MAX_SIZE	(BESBEV_ANA_BASE_ADDR+0x0B5)
/* TODO: check for reg max size 0xFF4 , default was 0xB4 */
#define BESBEV_DIGITAL_REGISTERS_MAX_SIZE	(BESBEV_DIG_BASE_ADDR+0xFF4)
#define BESBEV_ANALOG_MAX_REGISTER		(BESBEV_ANALOG_REGISTERS_MAX_SIZE - 1)
#define BESBEV_DIGITAL_MAX_REGISTER		(BESBEV_DIGITAL_REGISTERS_MAX_SIZE - 1)

#define BESBEV_ANA_BG_TSADC_BASE		(BESBEV_BASE+0x00000001)
#define BESBEV_REF_CTRL				(BESBEV_ANA_BG_TSADC_BASE+0x0000)
#define BESBEV_TEST_CTL_0			(BESBEV_ANA_BG_TSADC_BASE+0x0001)
#define BESBEV_BIAS_0				(BESBEV_ANA_BG_TSADC_BASE+0x0002)
#define BESBEV_OP_CTL				(BESBEV_ANA_BG_TSADC_BASE+0x0003)
#define BESBEV_IREF_CTL				(BESBEV_ANA_BG_TSADC_BASE+0x0004)
#define BESBEV_ISENS_CTL			(BESBEV_ANA_BG_TSADC_BASE+0x0005)
#define BESBEV_CLK_CTL				(BESBEV_ANA_BG_TSADC_BASE+0x0006)
#define BESBEV_TEST_CTL_1			(BESBEV_ANA_BG_TSADC_BASE+0x0007)
#define BESBEV_BIAS_1				(BESBEV_ANA_BG_TSADC_BASE+0x0008)
#define BESBEV_ADC_CTL				(BESBEV_ANA_BG_TSADC_BASE+0x0009)
#define BESBEV_DOUT_MSB				(BESBEV_ANA_BG_TSADC_BASE+0x000A)
#define BESBEV_DOUT_LSB				(BESBEV_ANA_BG_TSADC_BASE+0x000B)
#define BESBEV_VBAT_SNS				(BESBEV_ANA_BG_TSADC_BASE+0x000C)
#define BESBEV_ITRIM_CODE			(BESBEV_ANA_BG_TSADC_BASE+0x000D)

#define BESBEV_ANA_IVSENSE_BASE			(BESBEV_BASE+0x0000000F)
#define BESBEV_EN				(BESBEV_ANA_IVSENSE_BASE+0x0000)
#define BESBEV_OVERRIDE1			(BESBEV_ANA_IVSENSE_BASE+0x0001)
#define BESBEV_OVERRIDE2			(BESBEV_ANA_IVSENSE_BASE+0x0002)
#define BESBEV_VSENSE1				(BESBEV_ANA_IVSENSE_BASE+0x0003)
#define BESBEV_ISENSE1				(BESBEV_ANA_IVSENSE_BASE+0x0004)
#define BESBEV_ISENSE2				(BESBEV_ANA_IVSENSE_BASE+0x0005)
#define BESBEV_ISENSE_CAL			(BESBEV_ANA_IVSENSE_BASE+0x0006)
#define BESBEV_MISC				(BESBEV_ANA_IVSENSE_BASE+0x0007)
#define BESBEV_ADC_0				(BESBEV_ANA_IVSENSE_BASE+0x0008)
#define BESBEV_ADC_1				(BESBEV_ANA_IVSENSE_BASE+0x0009)
#define BESBEV_ADC_2				(BESBEV_ANA_IVSENSE_BASE+0x000A)
#define BESBEV_ADC_3				(BESBEV_ANA_IVSENSE_BASE+0x000B)
#define BESBEV_ADC_4				(BESBEV_ANA_IVSENSE_BASE+0x000C)
#define BESBEV_ADC_5				(BESBEV_ANA_IVSENSE_BASE+0x000D)
#define BESBEV_ADC_6				(BESBEV_ANA_IVSENSE_BASE+0x000E)
#define BESBEV_ADC_7				(BESBEV_ANA_IVSENSE_BASE+0x000F)
#define BESBEV_STATUS				(BESBEV_ANA_IVSENSE_BASE+0x0010)

#define BESBEV_ANA_SPK_TOP_BASE			(BESBEV_BASE+0x00000025)
#define BESBEV_DAC_CTRL_REG			(BESBEV_ANA_SPK_TOP_BASE+0x0000)
#define BESBEV_DAC_EN_DEBUG_REG			(BESBEV_ANA_SPK_TOP_BASE+0x0001)
#define BESBEV_DAC_OPAMP_BIAS1_REG		(BESBEV_ANA_SPK_TOP_BASE+0x0002)
#define BESBEV_DAC_OPAMP_BIAS2_REG		(BESBEV_ANA_SPK_TOP_BASE+0x0003)
#define BESBEV_DAC_VCM_CTRL_REG			(BESBEV_ANA_SPK_TOP_BASE+0x0004)
#define BESBEV_DAC_VOLTAGE_CTRL_REG		(BESBEV_ANA_SPK_TOP_BASE+0x0005)
#define BESBEV_ATEST1_REG			(BESBEV_ANA_SPK_TOP_BASE+0x0006)
#define BESBEV_ATEST2_REG			(BESBEV_ANA_SPK_TOP_BASE+0x0007)
#define BESBEV_SPKR_TOP_BIAS_REG1		(BESBEV_ANA_SPK_TOP_BASE+0x0008)
#define BESBEV_SPKR_TOP_BIAS_REG2		(BESBEV_ANA_SPK_TOP_BASE+0x0009)
#define BESBEV_SPKR_TOP_BIAS_REG3		(BESBEV_ANA_SPK_TOP_BASE+0x000A)
#define BESBEV_SPKR_TOP_BIAS_REG4		(BESBEV_ANA_SPK_TOP_BASE+0x000B)
#define BESBEV_SPKR_CLIP_DET_REG		(BESBEV_ANA_SPK_TOP_BASE+0x000C)
#define BESBEV_SPKR_DRV_LF_BLK_EN		(BESBEV_ANA_SPK_TOP_BASE+0x000D)
#define BESBEV_SPKR_DRV_LF_EN			(BESBEV_ANA_SPK_TOP_BASE+0x000E)
#define BESBEV_SPKR_DRV_LF_MASK_DCC_CTL		(BESBEV_ANA_SPK_TOP_BASE+0x000F)
#define BESBEV_SPKR_DRV_LF_MISC_CTL		(BESBEV_ANA_SPK_TOP_BASE+0x0010)
#define BESBEV_SPKR_DRV_LF_REG_GAIN		(BESBEV_ANA_SPK_TOP_BASE+0x0011)
#define BESBEV_SPKR_DRV_OS_CAL_CTL		(BESBEV_ANA_SPK_TOP_BASE+0x0012)
#define BESBEV_SPKR_DRV_OS_CAL_CTL1		(BESBEV_ANA_SPK_TOP_BASE+0x0013)
#define BESBEV_SPKR_PWM_CLK_CTL			(BESBEV_ANA_SPK_TOP_BASE+0x0014)
#define BESBEV_SPKR_PDRV_HS_CTL			(BESBEV_ANA_SPK_TOP_BASE+0x0015)
#define BESBEV_SPKR_PDRV_LS_CTL			(BESBEV_ANA_SPK_TOP_BASE+0x0016)
#define BESBEV_SPKR_PWRSTG_DBG			(BESBEV_ANA_SPK_TOP_BASE+0x0017)
#define BESBEV_SPKR_OCP_CTL			(BESBEV_ANA_SPK_TOP_BASE+0x0018)
#define BESBEV_SPKR_BBM_CTL			(BESBEV_ANA_SPK_TOP_BASE+0x0019)
#define BESBEV_PA_STATUS0			(BESBEV_ANA_SPK_TOP_BASE+0x001A)
#define BESBEV_PA_STATUS1			(BESBEV_ANA_SPK_TOP_BASE+0x001B)
#define BESBEV_PA_STATUS2			(BESBEV_ANA_SPK_TOP_BASE+0x001C)

#define BESBEV_ANA_BOOST_BASE			(BESBEV_BASE+0x00000043)
#define BESBEV_EN_CTRL				(BESBEV_ANA_BOOST_BASE+0x0000)
#define BESBEV_CURRENT_LIMIT			(BESBEV_ANA_BOOST_BASE+0x0001)
#define BESBEV_IBIAS1				(BESBEV_ANA_BOOST_BASE+0x0002)
#define BESBEV_IBIAS2				(BESBEV_ANA_BOOST_BASE+0x0003)
#define BESBEV_IBIAS3				(BESBEV_ANA_BOOST_BASE+0x0004)
#define BESBEV_LDO_PROG				(BESBEV_ANA_BOOST_BASE+0x0005)
#define BESBEV_STABILITY_CTRL1			(BESBEV_ANA_BOOST_BASE+0x0006)
#define BESBEV_STABILITY_CTRL2			(BESBEV_ANA_BOOST_BASE+0x0007)
#define BESBEV_PWRSTAGE_CTRL1			(BESBEV_ANA_BOOST_BASE+0x0008)
#define BESBEV_PWRSTAGE_CTRL2			(BESBEV_ANA_BOOST_BASE+0x0009)
#define BESBEV_BYPASS_1				(BESBEV_ANA_BOOST_BASE+0x000A)
#define BESBEV_BYPASS_2				(BESBEV_ANA_BOOST_BASE+0x000B)
#define BESBEV_ZX_CTRL_1			(BESBEV_ANA_BOOST_BASE+0x000C)
#define BESBEV_ZX_CTRL_2			(BESBEV_ANA_BOOST_BASE+0x000D)
#define BESBEV_MISC1				(BESBEV_ANA_BOOST_BASE+0x000E)
#define BESBEV_MISC2				(BESBEV_ANA_BOOST_BASE+0x000F)
#define BESBEV_GMAMP_SUP1			(BESBEV_ANA_BOOST_BASE+0x0010)
#define BESBEV_PWRSTAGE_CTRL3			(BESBEV_ANA_BOOST_BASE+0x0011)
#define BESBEV_PWRSTAGE_CTRL4			(BESBEV_ANA_BOOST_BASE+0x0012)
#define BESBEV_TEST1				(BESBEV_ANA_BOOST_BASE+0x0013)
#define BESBEV_SPARE1				(BESBEV_ANA_BOOST_BASE+0x0014)
#define BESBEV_SPARE2				(BESBEV_ANA_BOOST_BASE+0x0015)

#define BESBEV_ANA_PON_LDOL_BASE		(BESBEV_BASE+0x00000059)
#define BESBEV_PON_CTL_0			(BESBEV_ANA_PON_LDOL_BASE+0x0000)
#define BESBEV_PON_CLT_1			(BESBEV_ANA_PON_LDOL_BASE+0x0001)
#define BESBEV_PON_CTL_2			(BESBEV_ANA_PON_LDOL_BASE+0x0002)
#define BESBEV_PON_CTL_3			(BESBEV_ANA_PON_LDOL_BASE+0x0003)
#define BESBEV_CKWD_CTL_0			(BESBEV_ANA_PON_LDOL_BASE+0x0004)
#define BESBEV_CKWD_CTL_1			(BESBEV_ANA_PON_LDOL_BASE+0x0005)
#define BESBEV_CKWD_CTL_2			(BESBEV_ANA_PON_LDOL_BASE+0x0006)
#define BESBEV_CKSK_CTL_0			(BESBEV_ANA_PON_LDOL_BASE+0x0007)
#define BESBEV_PADSW_CTL_0			(BESBEV_ANA_PON_LDOL_BASE+0x0008)
#define BESBEV_TEST_0				(BESBEV_ANA_PON_LDOL_BASE+0x0009)
#define BESBEV_TEST_1				(BESBEV_ANA_PON_LDOL_BASE+0x000A)
#define BESBEV_STATUS_0				(BESBEV_ANA_PON_LDOL_BASE+0x000B)
#define BESBEV_STATUS_1				(BESBEV_ANA_PON_LDOL_BASE+0x000C)

#define BESBEV_DIG_CTRL_BASE			(BESBEV_BASE+0x00000400)
#define BESBEV_CHIP_ID0				(BESBEV_DIG_CTRL_BASE+0x0001)
#define BESBEV_CHIP_ID1				(BESBEV_DIG_CTRL_BASE+0x0002)
#define BESBEV_CHIP_ID2				(BESBEV_DIG_CTRL_BASE+0x0003)
#define BESBEV_CHIP_ID3				(BESBEV_DIG_CTRL_BASE+0x0004)
#define BESBEV_BUS_ID				(BESBEV_DIG_CTRL_BASE+0x0005)
#define BESBEV_CDC_RST_CTL			(BESBEV_DIG_CTRL_BASE+0x0006)
#define BESBEV_TOP_CLK_CFG			(BESBEV_DIG_CTRL_BASE+0x0007)
#define BESBEV_CDC_PATH_MODE			(BESBEV_DIG_CTRL_BASE+0x0008)
#define BESBEV_CDC_CLK_CTL			(BESBEV_DIG_CTRL_BASE+0x0009)
#define BESBEV_SWR_RESET_EN 			(BESBEV_DIG_CTRL_BASE+0x000A)
#define BESBEV_RESET_CTL			(BESBEV_DIG_CTRL_BASE+0x000B)
#define BESBEV_PA_FSM_CTL			(BESBEV_DIG_CTRL_BASE+0x0010)
#define BESBEV_PA_FSM_TIMER0			(BESBEV_DIG_CTRL_BASE+0x0011)
#define BESBEV_PA_FSM_TIMER1			(BESBEV_DIG_CTRL_BASE+0x0012)
#define BESBEV_PA_FSM_STA			(BESBEV_DIG_CTRL_BASE+0x0013)
#define BESBEV_PA_FSM_ERR_COND			(BESBEV_DIG_CTRL_BASE+0x0014)
#define BESBEV_PA_FSM_MSK			(BESBEV_DIG_CTRL_BASE+0x0015)
#define BESBEV_PA_FSM_BYP			(BESBEV_DIG_CTRL_BASE+0x0016)
#define BESBEV_PA_FSM_DBG			(BESBEV_DIG_CTRL_BASE+0x0017)
#define BESBEV_TADC_VALUE_CTL			(BESBEV_DIG_CTRL_BASE+0x0020)
#define BESBEV_TEMP_DETECT_CTL			(BESBEV_DIG_CTRL_BASE+0x0021)
#define BESBEV_TEMP_MSB				(BESBEV_DIG_CTRL_BASE+0x0022)
#define BESBEV_TEMP_LSB				(BESBEV_DIG_CTRL_BASE+0x0023)
#define BESBEV_TEMP_CONFIG0			(BESBEV_DIG_CTRL_BASE+0x0024)
#define BESBEV_TEMP_CONFIG1			(BESBEV_DIG_CTRL_BASE+0x0025)
#define BESBEV_VBAT_ADC_FLT_CTL			(BESBEV_DIG_CTRL_BASE+0x0026)
#define BESBEV_VBAT_DIN_MSB			(BESBEV_DIG_CTRL_BASE+0x0027)
#define BESBEV_VBAT_DIN_LSB			(BESBEV_DIG_CTRL_BASE+0x0028)
#define BESBEV_VBAT_DOUT			(BESBEV_DIG_CTRL_BASE+0x0029)
#define BESBEV_SDM_PDM9_LSB			(BESBEV_DIG_CTRL_BASE+0x002A)
#define BESBEV_SDM_PDM9_MSB			(BESBEV_DIG_CTRL_BASE+0x002B)
#define BESBEV_CDC_RX_CTL			(BESBEV_DIG_CTRL_BASE+0x0030)
#define BESBEV_CDC_SPK_DSM_A1_0			(BESBEV_DIG_CTRL_BASE+0x0031)
#define BESBEV_CDC_SPK_DSM_A1_1			(BESBEV_DIG_CTRL_BASE+0x0032)
#define BESBEV_CDC_SPK_DSM_A2_0			(BESBEV_DIG_CTRL_BASE+0x0033)
#define BESBEV_CDC_SPK_DSM_A2_1			(BESBEV_DIG_CTRL_BASE+0x0034)
#define BESBEV_CDC_SPK_DSM_A3_0			(BESBEV_DIG_CTRL_BASE+0x0035)
#define BESBEV_CDC_SPK_DSM_A3_1			(BESBEV_DIG_CTRL_BASE+0x0036)
#define BESBEV_CDC_SPK_DSM_A4_0			(BESBEV_DIG_CTRL_BASE+0x0037)
#define BESBEV_CDC_SPK_DSM_A4_1			(BESBEV_DIG_CTRL_BASE+0x0038)
#define BESBEV_CDC_SPK_DSM_A5_0			(BESBEV_DIG_CTRL_BASE+0x0039)
#define BESBEV_CDC_SPK_DSM_A5_1			(BESBEV_DIG_CTRL_BASE+0x003A)
#define BESBEV_CDC_SPK_DSM_A6_0			(BESBEV_DIG_CTRL_BASE+0x003B)
#define BESBEV_CDC_SPK_DSM_A7_0			(BESBEV_DIG_CTRL_BASE+0x003C)
#define BESBEV_CDC_SPK_DSM_C_0			(BESBEV_DIG_CTRL_BASE+0x003D)
#define BESBEV_CDC_SPK_DSM_C_1			(BESBEV_DIG_CTRL_BASE+0x003E)
#define BESBEV_CDC_SPK_DSM_C_2			(BESBEV_DIG_CTRL_BASE+0x003F)
#define BESBEV_CDC_SPK_DSM_C_3			(BESBEV_DIG_CTRL_BASE+0x0040)
#define BESBEV_CDC_SPK_DSM_R1			(BESBEV_DIG_CTRL_BASE+0x0041)
#define BESBEV_CDC_SPK_DSM_R2			(BESBEV_DIG_CTRL_BASE+0x0042)
#define BESBEV_CDC_SPK_DSM_R3			(BESBEV_DIG_CTRL_BASE+0x0043)
#define BESBEV_CDC_SPK_DSM_R4			(BESBEV_DIG_CTRL_BASE+0x0044)
#define BESBEV_CDC_SPK_DSM_R5			(BESBEV_DIG_CTRL_BASE+0x0045)
#define BESBEV_CDC_SPK_DSM_R6			(BESBEV_DIG_CTRL_BASE+0x0046)
#define BESBEV_CDC_SPK_DSM_R7			(BESBEV_DIG_CTRL_BASE+0x0047)
#define BESBEV_CDC_SPK_GAIN_PDM_0		(BESBEV_DIG_CTRL_BASE+0x0048)
#define BESBEV_CDC_SPK_GAIN_PDM_1		(BESBEV_DIG_CTRL_BASE+0x0049)
#define BESBEV_CDC_SPK_GAIN_PDM_2		(BESBEV_DIG_CTRL_BASE+0x004A)
#define BESBEV_PDM_WD_CTL			(BESBEV_DIG_CTRL_BASE+0x004B)
#define BESBEV_DEM_BYPASS_DATA0			(BESBEV_DIG_CTRL_BASE+0x004C)
#define BESBEV_DEM_BYPASS_DATA1			(BESBEV_DIG_CTRL_BASE+0x004D)
#define BESBEV_DEM_BYPASS_DATA2			(BESBEV_DIG_CTRL_BASE+0x004E)
#define BESBEV_DEM_BYPASS_DATA3			(BESBEV_DIG_CTRL_BASE+0x004F)
#define BESBEV_WAVG_CTL				(BESBEV_DIG_CTRL_BASE+0x0050)
#define BESBEV_WAVG_LRA_PER_0			(BESBEV_DIG_CTRL_BASE+0x0051)
#define BESBEV_WAVG_LRA_PER_1			(BESBEV_DIG_CTRL_BASE+0x0052)
#define BESBEV_WAVG_DELTA_THETA_0		(BESBEV_DIG_CTRL_BASE+0x0053)
#define BESBEV_WAVG_DELTA_THETA_1		(BESBEV_DIG_CTRL_BASE+0x0054)
#define BESBEV_WAVG_DIRECT_AMP_0		(BESBEV_DIG_CTRL_BASE+0x0055)
#define BESBEV_WAVG_DIRECT_AMP_1		(BESBEV_DIG_CTRL_BASE+0x0056)
#define BESBEV_WAVG_PTRN_AMP0_0			(BESBEV_DIG_CTRL_BASE+0x0057)
#define BESBEV_WAVG_PTRN_AMP0_1			(BESBEV_DIG_CTRL_BASE+0x0058)
#define BESBEV_WAVG_PTRN_AMP1_0			(BESBEV_DIG_CTRL_BASE+0x0059)
#define BESBEV_WAVG_PTRN_AMP1_1			(BESBEV_DIG_CTRL_BASE+0x005A)
#define BESBEV_WAVG_PTRN_AMP2_0			(BESBEV_DIG_CTRL_BASE+0x005B)
#define BESBEV_WAVG_PTRN_AMP2_1			(BESBEV_DIG_CTRL_BASE+0x005C)
#define BESBEV_WAVG_PTRN_AMP3_0			(BESBEV_DIG_CTRL_BASE+0x005D)
#define BESBEV_WAVG_PTRN_AMP3_1			(BESBEV_DIG_CTRL_BASE+0x005E)
#define BESBEV_WAVG_PTRN_AMP4_0			(BESBEV_DIG_CTRL_BASE+0x005F)
#define BESBEV_WAVG_PTRN_AMP4_1			(BESBEV_DIG_CTRL_BASE+0x0060)
#define BESBEV_WAVG_PTRN_AMP5_0			(BESBEV_DIG_CTRL_BASE+0x0061)
#define BESBEV_WAVG_PTRN_AMP5_1			(BESBEV_DIG_CTRL_BASE+0x0062)
#define BESBEV_WAVG_PTRN_AMP6_0			(BESBEV_DIG_CTRL_BASE+0x0063)
#define BESBEV_WAVG_PTRN_AMP6_1			(BESBEV_DIG_CTRL_BASE+0x0064)
#define BESBEV_WAVG_PTRN_AMP7_0			(BESBEV_DIG_CTRL_BASE+0x0065)
#define BESBEV_WAVG_PTRN_AMP7_1			(BESBEV_DIG_CTRL_BASE+0x0066)
#define BESBEV_WAVG_PER_0_1			(BESBEV_DIG_CTRL_BASE+0x0067)
#define BESBEV_WAVG_PER_2_3			(BESBEV_DIG_CTRL_BASE+0x0068)
#define BESBEV_WAVG_PER_4_5			(BESBEV_DIG_CTRL_BASE+0x0069)
#define BESBEV_WAVG_PER_6_7			(BESBEV_DIG_CTRL_BASE+0x006A)
#define BESBEV_WAVG_STA				(BESBEV_DIG_CTRL_BASE+0x006B)
#define BESBEV_DRE_CTL_0			(BESBEV_DIG_CTRL_BASE+0x006C)
#define BESBEV_DRE_CTL_1			(BESBEV_DIG_CTRL_BASE+0x006D)
#define BESBEV_DRE_IDLE_DET_CTL			(BESBEV_DIG_CTRL_BASE+0x006E)
#define BESBEV_CLSH_CTL_0			(BESBEV_DIG_CTRL_BASE+0x0070)
#define BESBEV_CLSH_CTL_1			(BESBEV_DIG_CTRL_BASE+0x0071)
#define BESBEV_CLSH_V_HD_PA			(BESBEV_DIG_CTRL_BASE+0x0072)
#define BESBEV_CLSH_V_PA_MIN			(BESBEV_DIG_CTRL_BASE+0x0073)
#define BESBEV_CLSH_OVRD_VAL			(BESBEV_DIG_CTRL_BASE+0x0074)
#define BESBEV_CLSH_HARD_MAX			(BESBEV_DIG_CTRL_BASE+0x0075)
#define BESBEV_CLSH_SOFT_MAX			(BESBEV_DIG_CTRL_BASE+0x0076)
#define BESBEV_CLSH_SIG_DP			(BESBEV_DIG_CTRL_BASE+0x0077)
#define BESBEV_TAGC_CTL				(BESBEV_DIG_CTRL_BASE+0x0078)
#define BESBEV_TAGC_TIME			(BESBEV_DIG_CTRL_BASE+0x0079)
#define BESBEV_TAGC_E2E_GAIN			(BESBEV_DIG_CTRL_BASE+0x007A)
#define BESBEV_TAGC_FORCE_VAL			(BESBEV_DIG_CTRL_BASE+0x007B)
#define BESBEV_VAGC_CTL				(BESBEV_DIG_CTRL_BASE+0x007C)
#define BESBEV_VAGC_TIME			(BESBEV_DIG_CTRL_BASE+0x007D)
#define BESBEV_VAGC_ATTN_LVL_1_2		(BESBEV_DIG_CTRL_BASE+0x007E)
#define BESBEV_VAGC_ATTN_LVL_3			(BESBEV_DIG_CTRL_BASE+0x007F)
#define BESBEV_INTR_MODE			(BESBEV_DIG_CTRL_BASE+0x0080)
#define BESBEV_INTR_MASK0			(BESBEV_DIG_CTRL_BASE+0x0081)
#define BESBEV_INTR_MASK1			(BESBEV_DIG_CTRL_BASE+0x0082)
#define BESBEV_INTR_STATUS0			(BESBEV_DIG_CTRL_BASE+0x0083)
#define BESBEV_INTR_STATUS1			(BESBEV_DIG_CTRL_BASE+0x0084)
#define BESBEV_INTR_CLEAR0			(BESBEV_DIG_CTRL_BASE+0x0085)
#define BESBEV_INTR_CLEAR1			(BESBEV_DIG_CTRL_BASE+0x0086)
#define BESBEV_INTR_LEVEL0			(BESBEV_DIG_CTRL_BASE+0x0087)
#define BESBEV_INTR_LEVEL1			(BESBEV_DIG_CTRL_BASE+0x0088)
#define BESBEV_INTR_SET0			(BESBEV_DIG_CTRL_BASE+0x0089)
#define BESBEV_INTR_SET1			(BESBEV_DIG_CTRL_BASE+0x008A)
#define BESBEV_INTR_TEST0			(BESBEV_DIG_CTRL_BASE+0x008B)
#define BESBEV_INTR_TEST1			(BESBEV_DIG_CTRL_BASE+0x008C)
#define BESBEV_OTP_CTRL0			(BESBEV_DIG_CTRL_BASE+0x0090)
#define BESBEV_OTP_CTRL1			(BESBEV_DIG_CTRL_BASE+0x0091)
#define BESBEV_HDRIVE_CTL_GROUP1		(BESBEV_DIG_CTRL_BASE+0x0092)
#define BESBEV_PIN_CTL				(BESBEV_DIG_CTRL_BASE+0x0093)
#define BESBEV_PIN_CTL_OE			(BESBEV_DIG_CTRL_BASE+0x0094)
#define BESBEV_PIN_WDATA_IOPAD			(BESBEV_DIG_CTRL_BASE+0x0095)
#define BESBEV_PIN_STATUS			(BESBEV_DIG_CTRL_BASE+0x0096)
#define BESBEV_I2C_SLAVE_CTL			(BESBEV_DIG_CTRL_BASE+0x0097)
#define BESBEV_PDM_TEST_MODE			(BESBEV_DIG_CTRL_BASE+0x00A0)
#define BESBEV_ATE_TEST_MODE			(BESBEV_DIG_CTRL_BASE+0x00A1)
#define BESBEV_DIG_DEBUG_MODE			(BESBEV_DIG_CTRL_BASE+0x00A3)
#define BESBEV_DIG_DEBUG_SEL			(BESBEV_DIG_CTRL_BASE+0x00A4)
#define BESBEV_DIG_DEBUG_EN			(BESBEV_DIG_CTRL_BASE+0x00A5)
#define BESBEV_SWR_HM_TEST0			(BESBEV_DIG_CTRL_BASE+0x00A6)
#define BESBEV_SWR_HM_TEST1			(BESBEV_DIG_CTRL_BASE+0x00A7)
#define BESBEV_SWR_PAD_CTL			(BESBEV_DIG_CTRL_BASE+0x00A8)
#define BESBEV_TADC_DETECT_DBG_CTL		(BESBEV_DIG_CTRL_BASE+0x00A9)
#define BESBEV_TADC_DEBUG_MSB			(BESBEV_DIG_CTRL_BASE+0x00AA)
#define BESBEV_TADC_DEBUG_LSB			(BESBEV_DIG_CTRL_BASE+0x00AB)
#define BESBEV_SAMPLE_EDGE_SEL			(BESBEV_DIG_CTRL_BASE+0x00AC)
#define BESBEV_SWR_EDGE_SEL			(BESBEV_DIG_CTRL_BASE+0x00AD)
#define BESBEV_TEST_MODE_CTL			(BESBEV_DIG_CTRL_BASE+0x00AE)
#define BESBEV_IOPAD_CTL			(BESBEV_DIG_CTRL_BASE+0x00AF)
#define BESBEV_ANA_CSR_DBG_ADD			(BESBEV_DIG_CTRL_BASE+0x00B0)
#define BESBEV_ANA_CSR_DBG_CTL			(BESBEV_DIG_CTRL_BASE+0x00B1)
#define BESBEV_SPARE_R				(BESBEV_DIG_CTRL_BASE+0x00BC)
#define BESBEV_SPARE_0				(BESBEV_DIG_CTRL_BASE+0x00BD)
#define BESBEV_SPARE_1				(BESBEV_DIG_CTRL_BASE+0x00BE)
#define BESBEV_SPARE_2				(BESBEV_DIG_CTRL_BASE+0x00BF)
#define BESBEV_SCODE				(BESBEV_DIG_CTRL_BASE+0x00C0)

#define BESBEV_DIG_TRIM_BASE			(BESBEV_BASE+0x00000500)
#define BESBEV_OTP_REG_0			(BESBEV_DIG_TRIM_BASE+0x0080)
#define BESBEV_OTP_REG_1			(BESBEV_DIG_TRIM_BASE+0x0081)
#define BESBEV_OTP_REG_2			(BESBEV_DIG_TRIM_BASE+0x0082)
#define BESBEV_OTP_REG_3			(BESBEV_DIG_TRIM_BASE+0x0083)
#define BESBEV_OTP_REG_4			(BESBEV_DIG_TRIM_BASE+0x0084)
#define BESBEV_OTP_REG_5			(BESBEV_DIG_TRIM_BASE+0x0085)
#define BESBEV_OTP_REG_6			(BESBEV_DIG_TRIM_BASE+0x0086)
#define BESBEV_OTP_REG_7			(BESBEV_DIG_TRIM_BASE+0x0087)
#define BESBEV_OTP_REG_8			(BESBEV_DIG_TRIM_BASE+0x0088)
#define BESBEV_OTP_REG_9			(BESBEV_DIG_TRIM_BASE+0x0089)
#define BESBEV_OTP_REG_10			(BESBEV_DIG_TRIM_BASE+0x008A)
#define BESBEV_OTP_REG_11			(BESBEV_DIG_TRIM_BASE+0x008B)
#define BESBEV_OTP_REG_12			(BESBEV_DIG_TRIM_BASE+0x008C)
#define BESBEV_OTP_REG_13			(BESBEV_DIG_TRIM_BASE+0x008D)
#define BESBEV_OTP_REG_14			(BESBEV_DIG_TRIM_BASE+0x008E)
#define BESBEV_OTP_REG_15			(BESBEV_DIG_TRIM_BASE+0x008F)
#define BESBEV_OTP_REG_16			(BESBEV_DIG_TRIM_BASE+0x0090)
#define BESBEV_OTP_REG_17			(BESBEV_DIG_TRIM_BASE+0x0091)
#define BESBEV_OTP_REG_18			(BESBEV_DIG_TRIM_BASE+0x0092)
#define BESBEV_OTP_REG_19			(BESBEV_DIG_TRIM_BASE+0x0093)
#define BESBEV_OTP_REG_20			(BESBEV_DIG_TRIM_BASE+0x0094)
#define BESBEV_OTP_REG_21			(BESBEV_DIG_TRIM_BASE+0x0095)
#define BESBEV_OTP_REG_22			(BESBEV_DIG_TRIM_BASE+0x0096)
#define BESBEV_OTP_REG_23			(BESBEV_DIG_TRIM_BASE+0x0097)
#define BESBEV_OTP_REG_24			(BESBEV_DIG_TRIM_BASE+0x0098)
#define BESBEV_OTP_REG_25			(BESBEV_DIG_TRIM_BASE+0x0099)
#define BESBEV_OTP_REG_26			(BESBEV_DIG_TRIM_BASE+0x009A)
#define BESBEV_OTP_REG_27			(BESBEV_DIG_TRIM_BASE+0x009B)
#define BESBEV_OTP_REG_28			(BESBEV_DIG_TRIM_BASE+0x009C)
#define BESBEV_OTP_REG_29			(BESBEV_DIG_TRIM_BASE+0x009D)
#define BESBEV_OTP_REG_30			(BESBEV_DIG_TRIM_BASE+0x009E)
#define BESBEV_OTP_REG_31			(BESBEV_DIG_TRIM_BASE+0x009F)
#define BESBEV_OTP_REG_32			(BESBEV_DIG_TRIM_BASE+0x00A0)
#define BESBEV_OTP_REG_33			(BESBEV_DIG_TRIM_BASE+0x00A1)
#define BESBEV_OTP_REG_34			(BESBEV_DIG_TRIM_BASE+0x00A2)
#define BESBEV_OTP_REG_35			(BESBEV_DIG_TRIM_BASE+0x00A3)
#define BESBEV_OTP_REG_63			(BESBEV_DIG_TRIM_BASE+0x00BF)

#define BESBEV_DIG_EMEM_BASE			(BESBEV_BASE+0x000005C0)
#define BESBEV_EMEM_0				(BESBEV_DIG_EMEM_BASE+0x0000)
#define BESBEV_EMEM_1				(BESBEV_DIG_EMEM_BASE+0x0001)
#define BESBEV_EMEM_2				(BESBEV_DIG_EMEM_BASE+0x0002)
#define BESBEV_EMEM_3				(BESBEV_DIG_EMEM_BASE+0x0003)
#define BESBEV_EMEM_4				(BESBEV_DIG_EMEM_BASE+0x0004)
#define BESBEV_EMEM_5				(BESBEV_DIG_EMEM_BASE+0x0005)
#define BESBEV_EMEM_6				(BESBEV_DIG_EMEM_BASE+0x0006)
#define BESBEV_EMEM_7				(BESBEV_DIG_EMEM_BASE+0x0007)
#define BESBEV_EMEM_8				(BESBEV_DIG_EMEM_BASE+0x0008)
#define BESBEV_EMEM_9				(BESBEV_DIG_EMEM_BASE+0x0009)
#define BESBEV_EMEM_10				(BESBEV_DIG_EMEM_BASE+0x000A)
#define BESBEV_EMEM_11				(BESBEV_DIG_EMEM_BASE+0x000B)
#define BESBEV_EMEM_12				(BESBEV_DIG_EMEM_BASE+0x000C)
#define BESBEV_EMEM_13				(BESBEV_DIG_EMEM_BASE+0x000D)
#define BESBEV_EMEM_14				(BESBEV_DIG_EMEM_BASE+0x000E)
#define BESBEV_EMEM_15				(BESBEV_DIG_EMEM_BASE+0x000F)
#define BESBEV_EMEM_16				(BESBEV_DIG_EMEM_BASE+0x0010)
#define BESBEV_EMEM_17				(BESBEV_DIG_EMEM_BASE+0x0011)
#define BESBEV_EMEM_18				(BESBEV_DIG_EMEM_BASE+0x0012)
#define BESBEV_EMEM_19				(BESBEV_DIG_EMEM_BASE+0x0013)
#define BESBEV_EMEM_20				(BESBEV_DIG_EMEM_BASE+0x0014)
#define BESBEV_EMEM_21				(BESBEV_DIG_EMEM_BASE+0x0015)
#define BESBEV_EMEM_22				(BESBEV_DIG_EMEM_BASE+0x0016)
#define BESBEV_EMEM_23				(BESBEV_DIG_EMEM_BASE+0x0017)
#define BESBEV_EMEM_24				(BESBEV_DIG_EMEM_BASE+0x0018)
#define BESBEV_EMEM_25				(BESBEV_DIG_EMEM_BASE+0x0019)
#define BESBEV_EMEM_26				(BESBEV_DIG_EMEM_BASE+0x001A)
#define BESBEV_EMEM_27				(BESBEV_DIG_EMEM_BASE+0x001B)
#define BESBEV_EMEM_28				(BESBEV_DIG_EMEM_BASE+0x001C)
#define BESBEV_EMEM_29				(BESBEV_DIG_EMEM_BASE+0x001D)
#define BESBEV_EMEM_30				(BESBEV_DIG_EMEM_BASE+0x001E)
#define BESBEV_EMEM_31				(BESBEV_DIG_EMEM_BASE+0x001F)
#define BESBEV_EMEM_32				(BESBEV_DIG_EMEM_BASE+0x0020)
#define BESBEV_EMEM_33				(BESBEV_DIG_EMEM_BASE+0x0021)
#define BESBEV_EMEM_34				(BESBEV_DIG_EMEM_BASE+0x0022)
#define BESBEV_EMEM_35				(BESBEV_DIG_EMEM_BASE+0x0023)
#define BESBEV_EMEM_36				(BESBEV_DIG_EMEM_BASE+0x0024)
#define BESBEV_EMEM_37				(BESBEV_DIG_EMEM_BASE+0x0025)
#define BESBEV_EMEM_38				(BESBEV_DIG_EMEM_BASE+0x0026)
#define BESBEV_EMEM_39				(BESBEV_DIG_EMEM_BASE+0x0027)
#define BESBEV_EMEM_40				(BESBEV_DIG_EMEM_BASE+0x0028)
#define BESBEV_EMEM_41				(BESBEV_DIG_EMEM_BASE+0x0029)
#define BESBEV_EMEM_42				(BESBEV_DIG_EMEM_BASE+0x002A)
#define BESBEV_EMEM_43				(BESBEV_DIG_EMEM_BASE+0x002B)
#define BESBEV_EMEM_44				(BESBEV_DIG_EMEM_BASE+0x002C)
#define BESBEV_EMEM_45				(BESBEV_DIG_EMEM_BASE+0x002D)
#define BESBEV_EMEM_46				(BESBEV_DIG_EMEM_BASE+0x002E)
#define BESBEV_EMEM_47				(BESBEV_DIG_EMEM_BASE+0x002F)
#define BESBEV_EMEM_48				(BESBEV_DIG_EMEM_BASE+0x0030)
#define BESBEV_EMEM_49				(BESBEV_DIG_EMEM_BASE+0x0031)
#define BESBEV_EMEM_50				(BESBEV_DIG_EMEM_BASE+0x0032)
#define BESBEV_EMEM_51				(BESBEV_DIG_EMEM_BASE+0x0033)
#define BESBEV_EMEM_52				(BESBEV_DIG_EMEM_BASE+0x0034)
#define BESBEV_EMEM_53				(BESBEV_DIG_EMEM_BASE+0x0035)
#define BESBEV_EMEM_54				(BESBEV_DIG_EMEM_BASE+0x0036)
#define BESBEV_EMEM_55				(BESBEV_DIG_EMEM_BASE+0x0037)
#define BESBEV_EMEM_56				(BESBEV_DIG_EMEM_BASE+0x0038)
#define BESBEV_EMEM_57				(BESBEV_DIG_EMEM_BASE+0x0039)
#define BESBEV_EMEM_58				(BESBEV_DIG_EMEM_BASE+0x003A)
#define BESBEV_EMEM_59				(BESBEV_DIG_EMEM_BASE+0x003B)
#define BESBEV_EMEM_60				(BESBEV_DIG_EMEM_BASE+0x003C)
#define BESBEV_EMEM_61				(BESBEV_DIG_EMEM_BASE+0x003D)
#define BESBEV_EMEM_62				(BESBEV_DIG_EMEM_BASE+0x003E)
#define BESBEV_EMEM_63				(BESBEV_DIG_EMEM_BASE+0x003F)

#define BESBEV_NUM_REGISTERS			(BESBEV_EMEM_63+1)
#define BESBEV_MAX_REGISTER			(BESBEV_NUM_REGISTERS-1)

#endif /* _BESBEV_REGISTERS_H */
