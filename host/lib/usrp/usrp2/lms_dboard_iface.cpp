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
#include <cstdio>
#include "lms_dboard_iface.hpp"
#include "usrp2_iface.hpp"

// Make Function

dboard_iface::sptr make_lms_dboard_iface(usrp2_iface::sptr iface) {
    return dboard_iface::sptr(new lms_dboard_iface(iface));
}

// test routines:
uint32_t lms_dboard_iface::read_addr(uint8_t lms, uint8_t addr) {
    if(addr > 127) return 0; // incorrect address, 7 bit long expected
    return _iface->read_spi(lms, spi_config_t::EDGE_RISE, addr<<8, 16);    
}

void lms_dboard_iface::write_addr_data(uint8_t lms, uint8_t addr, uint8_t data) {
    if(addr < 128) { // correct address is 7 bit long expected
        uint16_t command = ((uint16_t)addr << 8) | (uint16_t)data;
        _iface->write_spi(lms, spi_config_t::EDGE_RISE, command, 16);
    }
}

void lms_dboard_iface::brute_test() {
    for (int i = 0; i < 128; i++) {
        printf("i=%x LMS1=%x LMS2=%x\t", i, read_addr(1, i), read_addr(2, i));
	if(read_addr(1, i) == read_addr(2, i)) printf("OK\n"); else printf("DIFF\n");
    }
}
