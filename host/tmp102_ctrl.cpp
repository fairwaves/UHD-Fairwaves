//
// Copyright 2014 Fairwaves LLC
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

#include "tmp102_ctrl.hpp"
#include <boost/assign/list_of.hpp>

#include <iostream>

#if defined(_MSC_VER) && (_MSC_VER <= 1700)
#define nan(arg) std::strtod("NAN()", (char**)NULL)
#endif

using namespace uhd;

tmp102_ctrl::tmp102_ctrl()
    : _addr(TMP102_NONE)
    , _config(0)
{
}

bool tmp102_ctrl::check(const uhd::i2c_iface::sptr& iface, tmp102_addr addr)
{
    byte_vector_t cmd = boost::assign::list_of(REG_CONF);
    iface->write_i2c(addr, cmd);

    byte_vector_t vec = iface->read_i2c(addr, 2);
    uint16_t res = (((uint16_t)vec.at(0)) << 8) | vec.at(1);
    // Resolution bits are always set
    if (((res & CONF_R_MASK) != CONF_R_MASK) || ((res & 0xF) != 0)) {
        return false;
    }

    return true;
}

void tmp102_ctrl::init(i2c_iface::sptr iface, tmp102_addr addr)
{
    _iface = iface;
    _addr = addr;
    _config = get_reg(REG_CONF);
}

double tmp102_ctrl::get_temp()
{
    if (!_iface)
        return nan("");

    if (_config & CONF_SD) {
        // Start conversion in shutdown mode
        set_reg(REG_CONF, _config | CONF_OS);

        // Wait for conversion ready
        unsigned i;
        for (i = 0; i < TMP102_POLL_RDY_WATCHDOG; i++) {
            if (get_reg(REG_CONF) & CONF_OS)
                break;
        }

        if (i == TMP102_POLL_RDY_WATCHDOG)
            return nan("");
    }
    int16_t tmp = (int16_t)get_reg(REG_TEMP);
    if (tmp & 0x001) {
        tmp >>= 3;
    } else {
        tmp >>= 4;
    }
    return 0.0625 * (double)tmp;
}

void tmp102_ctrl::set_update_rate(conversion_rate rate)
{
    _config &= ~CONF_CR_MASK;
    _config |= (rate << CONF_CR_OFFS) & CONF_CR_MASK;
    set_reg(REG_CONF, _config);
}

void tmp102_ctrl::set_ex_mode(bool ex_mode)
{
    if (ex_mode)
        _config |= CONF_EM;
    else
        _config &= ~CONF_EM;
    set_reg(REG_CONF, _config);
}

void tmp102_ctrl::set_shutdown_mode(bool shutdown_mode)
{
    if (shutdown_mode)
        _config |= CONF_SD;
    else
        _config &= ~CONF_SD;
    set_reg(REG_CONF, _config);
}

uint16_t tmp102_ctrl::get_reg(tmp102_regs reg)
{
    byte_vector_t cmd = boost::assign::list_of(reg);
    _iface->write_i2c(_addr, cmd);

    byte_vector_t vec = _iface->read_i2c(_addr, 2);
    uint16_t res = ((uint16_t)vec.at(0) << 8) | vec.at(1);
    return res;
}

void tmp102_ctrl::set_reg(tmp102_regs reg, uint16_t value)
{
    byte_vector_t cmd = boost::assign::list_of((uint8_t)reg)((uint8_t)(value >> 8))((uint8_t)(value & 0xFF));
    _iface->write_i2c(_addr, cmd);
}
