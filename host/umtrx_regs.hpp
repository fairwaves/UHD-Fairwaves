//
// Copyright 2010-2011 Ettus Research LLC
// Copyright 2012 Fairwaves
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef INCLUDED_UMTRX_REGS_HPP
#define INCLUDED_UMTRX_REGS_HPP

#define localparam static const int

////////////////////////////////////////////////////////////////////////
// Define slave bases
////////////////////////////////////////////////////////////////////////
#define ROUTER_RAM_BASE     0x4000
#define SPI_BASE            0x5000
#define I2C_BASE            0x5400
#define GPIO_BASE           0x5800
#define READBACK_BASE       0x5C00
#define ETH_BASE            0x6000
#define SETTING_REGS_BASE   0x7000
#define PIC_BASE            0x8000
#define I2C_AUX_BASE        0x8400
#define UART_BASE           0x8800
#define ATR_BASE            0x8C00

////////////////////////////////////////////////////////////////////////
// Setting register offsets
////////////////////////////////////////////////////////////////////////
localparam SR_MISC     =   0;   // 9 regs
localparam SR_TIME64   =  10;   // 6
localparam SR_BUF_POOL =  16;   // 4

localparam SR_RX_FRONT0 =  20;   // 5
localparam SR_RX_FRONT1 =  25;   // 5
localparam SR_RX_CTRL0 =  30;   // 9
localparam SR_RX_DSP0  =  40;   // 7
localparam SR_RX_CTRL1 =  50;   // 9
localparam SR_RX_DSP1  =  60;   // 7
localparam SR_RX_CTRL2 =  70;   // 9
localparam SR_RX_DSP2  =  80;   // 7
localparam SR_RX_CTRL3 =  90;   // 9
localparam SR_RX_DSP3  =  100;   // 7

localparam SR_TX_FRONT0 = 110;   // ?
localparam SR_TX_CTRL0  = 126;   // 6
localparam SR_TX_DSP0   = 135;   // 5
localparam SR_TX_FRONT1 = 145;   // ?
localparam SR_TX_CTRL1  = 161;   // 6
localparam SR_TX_DSP1   = 170;   // 5

localparam SR_DIVSW    = 180;   // 2
localparam SR_RX_FE_SW = 183;   // 1
localparam SR_TX_FE_SW = 184;   // 1
localparam SR_SPI_CORE = 185;   // 3

#define U2_REG_SR_ADDR(sr) (SETTING_REGS_BASE + (4 * (sr)))

/////////////////////////////////////////////////
// SPI Slave Constants

////////////////////////////////////////////////
// Masks for controlling different peripherals in UmTRX
#define SPI_SS_LMS1    1
#define SPI_SS_LMS2    2
#define SPI_SS_DAC     4
#define SPI_SS_AUX1    8
#define SPI_SS_AUX2    16

/////////////////////////////////////////////////
// Misc Control
////////////////////////////////////////////////
#define U2_REG_MISC_LMS_RES U2_REG_SR_ADDR(0)
#define U2_REG_MISC_CTRL_SFC_CLEAR U2_REG_SR_ADDR(1)
#define U2_REG_MISC_CTRL_LEDS U2_REG_SR_ADDR(3)
#define U2_REG_MISC_CTRL_PHY U2_REG_SR_ADDR(4)
#define U2_REG_MISC_CTRL_DBG_MUX U2_REG_SR_ADDR(5)
#define U2_REG_MISC_CTRL_RAM_PAGE U2_REG_SR_ADDR(6)
#define U2_REG_MISC_CTRL_FLUSH_ICACHE U2_REG_SR_ADDR(7)

/////////////////////////////////////////////////
// Readback regs
////////////////////////////////////////////////
#define U2_REG_SPI_RB READBACK_BASE + 4*0
#define U2_REG_NUM_DDC READBACK_BASE + 4*1
#define U2_REG_NUM_DUC READBACK_BASE + 4*2
#define U2_REG_STATUS READBACK_BASE + 4*8
#define U2_REG_TIME64_HI_RB_IMM READBACK_BASE + 4*10
#define U2_REG_TIME64_LO_RB_IMM READBACK_BASE + 4*11
#define U2_REG_COMPAT_NUM_RB READBACK_BASE + 4*12
#define U2_REG_IRQ_RB READBACK_BASE + 4*13
#define U2_REG_TIME64_HI_RB_PPS READBACK_BASE + 4*14
#define U2_REG_TIME64_LO_RB_PPS READBACK_BASE + 4*15

#define AUX_LD1_IRQ_BIT (1 << 14)
#define AUX_LD2_IRQ_BIT (1 << 15)

/////////////////////////////////////////////////
// LMS regs
////////////////////////////////////////////////
#define LMS_BITS(val, shift, mask) (((val)<<(shift))&(mask))

#define LMS_DC_CAL_REG             0x33
#define LMS_DC_START_CLBR_SHIFT    5
#define LMS_DC_START_CLBR_MASK     (1<<LMS_DC_START_CLBR_SHIFT)
#define LMS_DC_LOAD_SHIFT          4
#define LMS_DC_LOAD_MASK           (1<<LMS_DC_LOAD_SHIFT)
#define LMS_DC_SRESET_SHIFT        3
#define LMS_DC_SRESET_MASK         (1<<LMS_DC_SRESET_SHIFT)
#define LMS_DC_ADDR_SHIFT          0
#define LMS_DC_ADDR_MASK           (7<<LMS_DC_ADDR_SHIFT)

// Defined for the U2_REG_MISC_LMS_CLOCK register
#define LMS1_RESET  (1<<0)
#define LMS2_RESET  (1<<1)

// Defined for the U2_REG_MISC_LMS_RES register
#define PAREG_NLOW_PA  (1<<2)
#define PAREG_ENPA1    (1<<3)
#define PAREG_ENPA2    (1<<4)
#define PAREG_ENDCSYNC (1<<5)

#endif
