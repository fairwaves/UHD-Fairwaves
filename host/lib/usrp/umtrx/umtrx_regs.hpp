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
#define UART_BASE           0x8800
#define ATR_BASE            0x8C00

////////////////////////////////////////////////////////////////////////
// Setting register offsets
////////////////////////////////////////////////////////////////////////
#define SR_MISC       0   // 7 regs
#define SR_SIMTIMER   8   // 2
#define SR_TIME64    10   // 6
#define SR_BUF_POOL  16   // 4

#define SR_RX_FRONT0  20   // 5
#define SR_RX_FRONT1  25   // 5

#define SR_RX_CTRL0  32   // 9
#define SR_RX_DSP0   48   // 7
#define SR_RX_CTRL1  80   // 9
#define SR_RX_DSP1   96   // 7

#define SR_TX_FRONT0 110   // ?
#define SR_TX_CTRL0  126   // 6
#define SR_TX_DSP0   135   // 5
#define SR_TX_FRONT1 145   // ?
#define SR_TX_CTRL1  161   // 6
#define SR_TX_DSP1   170   // 5

// DSPs to frontends mapping controls
#define SR_RX_FRONT_SW 176 // 1
#define SR_TX_FRONT_SW 177 // 1

// Registers for controlling diversity switches
#define SR_DIVSW   180 // 2

#define SR_GPIO     184
#define SR_UDP_SM   192   // 64

#define U2_REG_SR_ADDR(sr) (SETTING_REGS_BASE + (4 * (sr)))

#define U2_REG_ROUTER_CTRL_PORTS U2_REG_SR_ADDR(SR_BUF_POOL) + 8

/////////////////////////////////////////////////
// SPI Slave Constants

////////////////////////////////////////////////
// Masks for controlling different peripherals
#define SPI_SS_RX_DB    16
#define SPI_SS_TX_DB   128

////////////////////////////////////////////////
// Masks for controlling different peripherals in UmTRX
#define SPI_SS_LMS1    1
#define SPI_SS_LMS2    2
#define SPI_SS_DAC     4

/////////////////////////////////////////////////
// Misc Control
////////////////////////////////////////////////
#define U2_REG_MISC_CTRL_CLOCK U2_REG_SR_ADDR(0)
#define U2_REG_MISC_CTRL_SERDES U2_REG_SR_ADDR(1)
#define U2_REG_MISC_CTRL_ADC U2_REG_SR_ADDR(2)
#define U2_REG_MISC_CTRL_LEDS U2_REG_SR_ADDR(3)
#define U2_REG_MISC_CTRL_PHY U2_REG_SR_ADDR(4)
#define U2_REG_MISC_CTRL_DBG_MUX U2_REG_SR_ADDR(5)
#define U2_REG_MISC_CTRL_RAM_PAGE U2_REG_SR_ADDR(6)
#define U2_REG_MISC_CTRL_FLUSH_ICACHE U2_REG_SR_ADDR(7)
#define U2_REG_MISC_CTRL_LED_SRC U2_REG_SR_ADDR(8)

#define U2_FLAG_MISC_CTRL_SERDES_ENABLE 8
#define U2_FLAG_MISC_CTRL_SERDES_PRBSEN 4
#define U2_FLAG_MISC_CTRL_SERDES_LOOPEN 2
#define U2_FLAG_MISC_CTRL_SERDES_RXEN   1

#define U2_FLAG_MISC_CTRL_ADC_ON  0x0F
#define U2_FLAG_MISC_CTRL_ADC_OFF 0x00

/////////////////////////////////////////////////
// Readback regs
////////////////////////////////////////////////
#define U2_REG_STATUS READBACK_BASE + 4*8
#define U2_REG_GPIO_RB READBACK_BASE + 4*9
#define U2_REG_TIME64_SECS_RB_IMM READBACK_BASE + 4*10
#define U2_REG_TIME64_TICKS_RB_IMM READBACK_BASE + 4*11
#define U2_REG_COMPAT_NUM_RB READBACK_BASE + 4*12
#define U2_REG_IRQ_RB READBACK_BASE + 4*13
#define U2_REG_TIME64_SECS_RB_PPS READBACK_BASE + 4*14
#define U2_REG_TIME64_TICKS_RB_PPS READBACK_BASE + 4*15

#endif
