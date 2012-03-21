//
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

#endif
