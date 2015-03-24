/*
 * Copyright 2012 Alexander Chemeris <Alexander.Chemeris@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "umtrx_init.h"
#include "memory_map.h"
#include "spi.h"
#include "gpsdo.h"
//#include "pic.h"
//#include "hal_io.h"
//#include "hal_uart.h"
//#include "i2c.h"
#include "mdelay.h"
//#include "clocks.h"
//#include "usrp2/fw_common.h"
#include "nonstdio.h"

/*
 * UmTRX specific initialization steps
 */
bool
umtrx_init(void)
{
  uint32_t res;

//issue a reset to the LMS chips
  output_regs->lms_res = (LMS1_RESET | LMS2_RESET); // reset pins of lms chips switched to 1
  mdelay(100);
  output_regs->lms_res = 0; // reset pins of lms chips switched to 0
  mdelay(100);
  output_regs->lms_res = (LMS1_RESET | LMS2_RESET); // reset pins of lms chips switched to 1

  // Check LMS presense
  res = spi_transact(SPI_TXRX, SPI_SS_LMS1, LMS_RD_CMD(0x04), 16, SPI_PUSH_FALL|SPI_LATCH_RISE);
  printf("LMS1 chip version = 0x%x\n", res);
  res = spi_transact(SPI_TXRX, SPI_SS_LMS2, LMS_RD_CMD(0x04), 16, SPI_PUSH_FALL|SPI_LATCH_RISE);
  printf("LMS2 chip version = 0x%x\n", res);

  // Init GPSDO
  gpsdo_init();

/*
  // Enable RX and TX
  putstr("\nEnabling Tx and Rx on LMS1 and LMS2\n");
  // register 5:
  //   DECODE 0, SRESET 1, EN 1, STXEN 1, SRXEN 1, TFWMODE 1
  val = 0x3E;
  spi_transact(SPI_TXONLY, SPI_SS_LMS1, LMS_WR_CMD(0x05, val), 16, SPIF_PUSH_FALL|SPIF_LATCH_RISE);
  res = spi_transact(SPI_TXRX, SPI_SS_LMS1, LMS_RD_CMD(0x05), 16, SPIF_PUSH_FALL|SPIF_LATCH_RISE);
  printf("LMS1 register readback = 0x%x\n", res);
  spi_transact(SPI_TXONLY, SPI_SS_LMS2, LMS_WR_CMD(0x05, val), 16, SPIF_PUSH_FALL|SPIF_LATCH_RISE);
  res = spi_transact(SPI_TXRX, SPI_SS_LMS2, LMS_RD_CMD(0x05), 16, SPIF_PUSH_FALL|SPIF_LATCH_RISE);
  printf("LMS2 register readback = 0x%x\n", res);
*/
  return true;
}
