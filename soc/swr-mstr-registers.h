/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _SWRM_REGISTERS_H
#define _SWRM_REGISTERS_H

#define SWRM_BASE                                 0x00
#define SWRM_COMP_HW_VERSION                      (SWRM_BASE+0x0000)
#define SWRM_COMP_CFG                             (SWRM_BASE+0x0004)
#define SWRM_COMP_SW_RESET                        (SWRM_BASE+0x0008)
#define SWRM_COMP_POWER_CFG                       (SWRM_BASE+0x000C)
#define SWRM_COMP_FEATURE_CFG                     (SWRM_BASE+0x0010)
#define SWRM_COMP_STATUS                          (SWRM_BASE+0x0014)
#define SWRM_LINK_MANAGER_EE                      (SWRM_BASE+0x0018)
#define SWRM_COMP_PARAMS                          (SWRM_BASE+0x0100)
#define SWRM_COMP_MASTER_ID                       (SWRM_BASE+0x0104)
#define MM_SYNC_CONFIG                            (SWRM_BASE+0x0108)
#define SWRM_COMP_NPL_PARAMS                      (SWRM_BASE+0x0120)

#ifdef CONFIG_SWRM_VER_2P0
#define SWRM_INTERRUPT_STATUS(n)          (SWRM_BASE+0x4000+0x1000*n)
#define SWRM_INTERRUPT_EN(n)              (SWRM_BASE+0x4004+0x1000*n)
#define SWRM_INTERRUPT_CLEAR(n)           (SWRM_BASE+0x4008+0x1000*n)
#define SWRM_CMD_FIFO_WR_CMD(n)           (SWRM_BASE+0x4020+0x1000*n)
#define SWRM_CMD_FIFO_RD_CMD(n)           (SWRM_BASE+0x4024+0x1000*n)
#define SWRM_NUM_ENTRIES_CMD_FIFO(n)      (SWRM_BASE+0x4028+0x1000*n)
#define SWRM_CMD_FIFO_RD_FIFO(n)          (SWRM_BASE+0x4040+0x1000*n)
#define SWRM_NUM_ENTRIES_RD_FIFO(n)       (SWRM_BASE+0x4044+0x1000*n)
#define SWRM_CMD_FIFO_STATUS(n)           (SWRM_BASE+0x4050+0x1000*n)
#define SWRM_CMD_RESPONSE(n)              (SWRM_BASE+0x4054+0x1000*n)
#define SWRM_CLK_CTRL(n)                  (SWRM_BASE+0x4060+0x1000*n)
#define SWRM_LINK_STATUS(n)               (SWRM_BASE+0x4064+0x1000*n)
#define SWRM_TO_CPU_SW_MESSAGE(n, m)      (SWRM_BASE+0x4090+0x1000*n+0x4*m)
#define SWRM_TO_CPU_SW_MESSAGE_READ(n, m) (SWRM_BASE+0x40A0+0x1000*n+0x4*m)
#define SWRM_DOUT_DP_FIFO_WATERMARK_CTRL(n) (SWRM_BASE+0x1060+0x100*n)
#define SWRM_INTERRUPT_STATUS_RD_FIFO_OVERFLOW                  0x10
#define SWRM_INTERRUPT_STATUS_RD_FIFO_UNDERFLOW                 0x20
#define SWRM_INTERRUPT_STATUS_SPECIAL_CMD_ID_FINISHED           0x400
#define SWRM_INTERRUPT_STATUS_MASK                              0x1DFDFD
#define SWRM_INTERRUPT_MAX                                      0x17

#else
#define SWRM_INTERRUPT_STATUS(n)                  (SWRM_BASE+0x0200*n)
#define SWRM_INTERRUPT_CLEAR(n)                   (SWRM_BASE+0x0208*n)
#define SWRM_INTERRUPT_STATUS_1(n)                (SWRM_BASE+0x0220*n)
#define SWRM_INTERRUPT_CLEAR_1(n)                 (SWRM_BASE+0x0228*n)
#define SWRM_CPU1_INTERRUPT_EN(n)                 (SWRM_BASE+0x0210*n)
#define SWRM_CPU1_INTERRUPT_EN_1(n)               (SWRM_BASE+0x0230*n)
#define SWRM_CPU0_CMD_RESPONSE(n)                 (SWRM_BASE+0x0250*n)

#define SWRM_CPU1_CMD_FIFO_WR_CMD(n)              (SWRM_BASE+0x031C*n)
#define SWRM_CPU1_CMD_FIFO_RD_CMD(n)              (SWRM_BASE+0x0320*n)
#define SWRM_CPU1_CMD_FIFO_STATUS(n)              (SWRM_BASE+0x0328*n)
#define SWRM_CPU1_CMD_FIFO_RD_FIFO(n)             (SWRM_BASE+0x0334*n)

#ifdef CONFIG_SWRM_VER_1P7
#define SWRM_INTERRUPT_EN                    SWRM_CPU1_INTERRUPT_EN
#define SWRM_INTERRUPT_EN_1                  SWRM_CPU1_INTERRUPT_EN_1
#define SWRM_CMD_FIFO_WR_CMD                 SWRM_CPU1_CMD_FIFO_WR_CMD
#define SWRM_CMD_FIFO_RD_CMD                 SWRM_CPU1_CMD_FIFO_RD_CMD
#define SWRM_CMD_FIFO_STATUS                 SWRM_CPU1_CMD_FIFO_STATUS
#define SWRM_CMD_FIFO_RD_FIFO                SWRM_CPU1_CMD_FIFO_RD_FIFO
#define SWRM_INTERRUPT_STATUS_MASK                              0x1DFDFD
#define SWRM_INTERRUPT_STATUS_RD_FIFO_OVERFLOW                  0x40000
#define SWRM_INTERRUPT_STATUS_RD_FIFO_UNDERFLOW                 0x80000
#define SWRM_INTERRUPT_STATUS_SPECIAL_CMD_ID_FINISHED           0x100000
#define SWRM_INTERRUPT_MAX                                      0x20
#else
#define SWRM_INTERRUPT_EN(n)                      SWRM_CPU1_INTERRUPT_EN(n)
#define SWRM_INTERRUPT_EN_1(n)                    (SWRM_BASE+0x0224*n)
#define SWRM_CMD_FIFO_WR_CMD(n)                   (SWRM_BASE+0x0300*n)
#define SWRM_CMD_FIFO_RD_CMD(n)                   (SWRM_BASE+0x0304*n)
#define SWRM_CMD_FIFO_STATUS(n)                   (SWRM_BASE+0x030C*n)
#define SWRM_CMD_FIFO_RD_FIFO(n)                  (SWRM_BASE+0x0318*n)
#define SWRM_INTERRUPT_STATUS_MASK                              0x1FDFD
#define SWRM_INTERRUPT_STATUS_RD_FIFO_OVERFLOW                  0x10
#define SWRM_INTERRUPT_STATUS_RD_FIFO_UNDERFLOW                 0x20
#define SWRM_INTERRUPT_STATUS_SPECIAL_CMD_ID_FINISHED           0x400
#define SWRM_INTERRUPT_MAX                                      0x11
#endif /* CONFIG_SWRM_VER_1P7 */
#endif /* CONFIG_SWRM_VER_2P0 */
#define SWRM_CMD_FIFO_CMD                         (SWRM_BASE+0x0308)
#define SWRM_CMD_FIFO_CFG                         (SWRM_BASE+0x0314)
#define SWRM_CPU_NUM_ENTRIES_WR_CMD_FIFO          (SWRM_BASE+0x0370)
#define SWRM_CPU0_SW_INTERRUPT_SET                (SWRM_BASE+0x0374)
#define SWRM_CPU0_SW_MESSAGE0                     (SWRM_BASE+0x0384)
#define SWRM_CPU0_SW_MESSAGE1                     (SWRM_BASE+0x0394)
#define SWRM_ENUMERATOR_CFG                       (SWRM_BASE+0x0500)
#define SWRM_ENUMERATOR_STATUS                    (SWRM_BASE+0x0504)
#define SWRM_ENUMERATOR_PRE_ENUM_CFG              (SWRM_BASE+0x0530)
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_1(m)         (SWRM_BASE+0x0530+0x8*m)
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_2(m)         (SWRM_BASE+0x0534+0x8*m)
#define SWRM_CTRL_W_GEN_STATUS                    (SWRM_BASE+0x0600)
#define SWRM_SW_RESET_STATUS                      (SWRM_BASE+0x0700)
#define SWRM_FORCE_BANK_SWITCH_SUCCESS            (SWRM_BASE+0x0704)
#define SWRM_SILENCE_TONE_REPEAT_VALUE_THRESHOLD  (SWRM_BASE+0x0710)
#define SWRM_SELF_GENERATE_FRAME_SYNC             (SWRM_BASE+0x0714)
#define SWRM_MCP_FRAME_CTRL_BANK(m)               (SWRM_BASE+0x101C+0x40*m)
#define SWRM_MCP_BUS_CTRL                         (SWRM_BASE+0x1044)
#define SWRM_MCP_CFG                              (SWRM_BASE+0x1048)
#define SWRM_MCP_STATUS                           (SWRM_BASE+0x104C)
#define SWRM_MCP_SLV_STATUS                       (SWRM_BASE+0x1090)


#define SWRM_DP_PORT_CONTROL(n)           (SWRM_BASE+0x1020+0x100*n)
#define SWRM_DP_PORT_CTRL_BANK(n, m)      (SWRM_BASE+0x1024+0x100*n+0x40*m)
#define SWRM_DP_PORT_CTRL_2_BANK(n, m)    (SWRM_BASE+0x1028+0x100*n+0x40*m)
#define SWRM_DP_BLOCK_CTRL_1(n)           (SWRM_BASE+0x102C+0x100*n)
#define SWRM_DP_BLOCK_CTRL2_BANK(n, m)    (SWRM_BASE+0x1030+0x100*n+0x40*m)
#define SWRM_DP_PORT_HCTRL_BANK(n, m)     (SWRM_BASE+0x1034+0x100*n+0x40*m)
#define SWRM_DP_BLOCK_CTRL3_BANK(n, m)    (SWRM_BASE+0x1038+0x100*n+0x40*m)
#define SWRM_DP_SAMPLECTRL2_BANK(n, m)    (SWRM_BASE+0x103C+0x100*n+0x40*m)


#define SWRM_DOUT_DP_INT_STATUS(n)          (SWRM_BASE+0x1000+0x100*n)
#define SWRM_DOUT_DP_INT_CLEAR(n)           (SWRM_BASE+0x1008+0x100*n)
#define SWRM_DOUT_DP_FEATURES_EN(n)         (SWRM_BASE+0x104C+0x100*n)
#define SWRM_DOUT_DP_SILENCE_TONE_CFG(n)    (SWRM_BASE+0x1050+0x100*n)
#define SWRM_DOUT_DP_PCM_PORT_CTRL(n)       (SWRM_BASE+0x1054+0x100*n)

#define SWRM_DIN_DP_INT_STATUS(n)         (SWRM_BASE+0x1000+0x100*n)
#define SWRM_DIN_DP_INT_CLEAR(n)          (SWRM_BASE+0x1008+0x100*n)
#define SWRM_DIN_DP_FEATURES_EN(n)        (SWRM_BASE+0x104C+0x100*n)
#define SWRM_DIN_DP_PCM_PORT_CTRL(n)      (SWRM_BASE+0x1054+0x100*n)

#ifdef CONFIG_SWRM_VER_2P0
#define SWRM_MAX_REGISTER SWRM_TO_CPU_SW_MESSAGE_READ(2, 2)
#else
#define SWRM_MAX_REGISTER SWRM_DIN_DP_PCM_PORT_CTRL(9)
#endif /* CONFIG_SWRM_VER_2P0 */


#define SWRM_INTERRUPT_STATUS_SLAVE_PEND_IRQ                    0x1
#define SWRM_INTERRUPT_STATUS_NEW_SLAVE_ATTACHED                0x2
#define SWRM_INTERRUPT_STATUS_CHANGE_ENUM_SLAVE_STATUS          0x4
#define SWRM_INTERRUPT_STATUS_MASTER_CLASH_DET                  0x8
#define SWRM_INTERRUPT_STATUS_WR_CMD_FIFO_OVERFLOW              0x40
#define SWRM_INTERRUPT_STATUS_CMD_ERROR                         0x80
#define SWRM_INTERRUPT_STATUS_DOUT_PORT_COLLISION               0x100
#define SWRM_INTERRUPT_STATUS_READ_EN_RD_VALID_MISMATCH         0x200

#ifdef CONFIG_SWRM_VER_1P1
#define SWRM_INTERRUPT_STATUS_NEW_SLAVE_AUTO_ENUM_FINISHED   0x800
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_FAILED               0x1000
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_TABLE_IS_FULL        0x2000
#define SWRM_INTERRUPT_STATUS_BUS_RESET_FINISHED             0x4000
#define SWRM_INTERRUPT_STATUS_CLK_STOP_FINISHED              0x8000
#define SWRM_INTERRUPT_STATUS_ERROR_PORT_TEST                0x10000
#else
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_FAILED               0x800
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_TABLE_IS_FULL        0x1000
#define SWRM_INTERRUPT_STATUS_BUS_RESET_FINISHED             0x2000
#define SWRM_INTERRUPT_STATUS_CLK_STOP_FINISHED              0x4000
#define SWRM_INTERRUPT_STATUS_ERROR_PORT_TEST                0x8000
#endif /* CONFIG_SWRM_VER_1P1 */

#define SWRM_INTERRUPT_STATUS_EXT_CLK_STOP_WAKEUP            0x10000
#define SWRM_INTERRUPT_STATUS_DOUT_RATE_MISMATCH             0x20000
#define SWRM_INTERRUPT_STATUS_BUS_REL_REQUEST_NON_MONITOR_MODE 0x40000
#define SWRM_INTERRUPT_STATUS_CMD_IGNORED_AND_EXEC_CONTINUED 0x80000
#define SWRM_INTERRUPT_STATUS_SW_INTERRUPT_FROM_CPU0         0x100000
#define SWRM_INTERRUPT_STATUS_SW_INTERRUPT_FROM_CPU1         0x200000
#define SWRM_INTERRUPT_STATUS_SW_INTERRUPT_FROM_CPU2         0x400000
#define SWRM_INTERRUPT_STATUS_SW_INTERRUPT_FROM_CPU3         0x800000


#define SWRM_COMP_PARAMS_WR_FIFO_DEPTH                       0x00007C00
#define SWRM_COMP_PARAMS_RD_FIFO_DEPTH                       0x000F8000

#define SWRM_COMP_FEATURE_CFG_DEFAULT_VAL                    0x06
#define SWRM_COMP_FEATURE_CFG_DEFAULT_VAL_V1P7               0x406
#define SWRM_COMP_FEATURE_CFG_DEFAULT_VAL_V2P0               0x40E
#define SWRM_COMP_FEATURE_CFG_PCM_EN_MASK                    0x18
#define SWRM_COMP_PARAMS_AUTO_ENUM_SLAVES                    0x00F00000
#endif /* _SWRM_REGISTERS_H */
