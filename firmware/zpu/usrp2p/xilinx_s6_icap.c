/* -*- c -*- */
/*
 * Copyright 2009-2011 Ettus Research LLC
 * Copyright 2012 Alexander Chemeris <Alexander.Chemeris@gmail.com>
 * Copyright 2013 Andrew Karpenkov <Andrew.Karpenkov@gmail.com>
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


#include <xilinx_s6_icap.h>
#include <memory_map.h>
#include <spi_flash_private.h> //for READ_CMD
#include "mdelay.h"

/* bit swap end-for-end, 8 bit */
static inline unsigned char
swap8(unsigned char x)
{
  unsigned char r = 0;
  r |= (x >> 7) & 0x01;
  r |= (x >> 5) & 0x02;
  r |= (x >> 3) & 0x04;
  r |= (x >> 1) & 0x08;

  r |= (x << 1) & 0x10;
  r |= (x << 3) & 0x20;
  r |= (x << 5) & 0x40;
  r |= (x << 7) & 0x80;

  return r;
}

/* bit swap end-for-end, 16 bit */
static inline uint16_t
swap16(uint16_t x)
{
  return (uint16_t)swap8(x&0xFF) | ((uint16_t)swap8((x>>8)&0xFF))<<8;
}

static inline void
wr_icap(uint16_t x)
{
    icap_regs->icap = swap16(x);
    mdelay(10);
}

static inline uint16_t
rd_icap(void)
{
    return swap16(icap_regs->icap);
}

uint16_t icap_s6_read_stat()
{
    uint16_t stat;

    //UG380 p108
    wr_icap(0xffff); //dummy word
    wr_icap(0xffff); //dummy word
    wr_icap(0xAA99); //sync word
    wr_icap(0x5566); //sync word
    wr_icap(0x2000); //NOOP
    wr_icap(0x2901); //Write Type1 packet header to read STAT register
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP
    stat = rd_icap(); //Device writes one word from the STAT register to the configuration interface
    wr_icap(0x30A1); //Type 1 Write 1 Word to CMD
    wr_icap(0x000D); //DESYNC Command
    wr_icap(0x2000); //NOOP
    wr_icap(0x2000); //NOOP

    return stat;
}

void
icap_s6_reload_fpga(uint32_t flash_address, uint32_t fallback_flash_address)
{
    //note! t.c[0] MUST contain the byte-wide read command for the flash device used.
    //for the 25P64, and most other flash devices, this is 0x03.
    union {
        uint32_t i;
        uint16_t w[2];
        uint8_t  c[4];
    } t, s;

    t.i = flash_address;
    t.c[0] = FAST_READ_CMD;
    s.i = fallback_flash_address;
    s.c[0] = FAST_READ_CMD;

    //TODO: look up the watchdog timer, ensure it won't fire too soon

    //UG380 p126
    wr_icap(0xffff); //dummy word
    wr_icap(0xffff); //dummy word
    wr_icap(0xAA99); //sync word
    wr_icap(0x5566); //sync word
    wr_icap(0x3261); //Type 1 write General 1 (1 word)
    wr_icap(t.w[1]); //MultiBoot Start Address [15:0]
    wr_icap(0x3281); //Type 1 write General 2 (1 word)
    wr_icap((t.c[0]<<8)|t.c[1]); //MultiBoot Start Address [23:16] and the byte-wide read command
    wr_icap(0x32A1); //Type 1 write General 3 (1 word)
    wr_icap(s.w[1]); //Fallback Start Addres [15:0]
    wr_icap(0x32C1); //Type 1 write General 4 (1 word)
    wr_icap((s.c[0]<<8)|s.c[1]); //Fallback Start Address [23:16] and the byte-wide read command
    wr_icap(0x30A1); //Type 1 write CMD (1 word)
    wr_icap(0x0000);
    wr_icap(0x30A1); //Type 1 write CMD (1 word)
    wr_icap(0x000E); //REBOOT command
    wr_icap(0x2000); //Type 1 NOP
    wr_icap(0x2000);
    wr_icap(0x2000);
    wr_icap(0x2000);
}
