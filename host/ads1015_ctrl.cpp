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

#include "ads1015_ctrl.hpp"
#include <boost/assign/list_of.hpp>

#include <iostream>

#if defined(_MSC_VER) && (_MSC_VER <= 1700)
#define nan(arg) std::strtod("NAN()", (char**)NULL)
#endif

using namespace uhd;

ads1015_ctrl::ads1015_ctrl()
    : _addr(ADS1015_NONE)
    , _config_reg(0)
{

}

bool ads1015_ctrl::check(const uhd::i2c_iface::sptr& iface, ads1015_addr addr)
{
    byte_vector_t cmd = boost::assign::list_of(ADS1015_REG_LO_THRES);
    iface->write_i2c(addr, cmd);

    byte_vector_t vec = iface->read_i2c(addr, 2);
    uint16_t res = (((uint16_t)vec.at(0)) << 8) | vec.at(1);
    if (res == 0 || ((res & 0xF) != 0)) {
        return false;
    }

    cmd[0] = ADS1015_REG_HI_THRES;
    iface->write_i2c(addr, cmd);
    vec = iface->read_i2c(addr, 2);
    res = (((uint16_t)vec.at(0)) << 8) | vec.at(1);
    if (res == 0xFFFF || ((res & 0xF) != 0xF)) {
        return false;
    }

    return true;
}

void ads1015_ctrl::init(i2c_iface::sptr iface, ads1015_addr addr)
{
    _iface = iface;
    _addr = addr;
    _config_reg = get_reg(ADS1015_REG_CONFIG);
}

void ads1015_ctrl::set_input(ads1015_input input)
{
    _config_reg = (_config_reg & ~ADS1015_CONF_MUX_MASK) |
                  (input << ADS1015_CONF_MUX_OFFS);
    set_reg(ADS1015_REG_CONFIG, _config_reg);
}

void ads1015_ctrl::set_pga(ads1015_pga pga)
{
    _config_reg = (_config_reg & ~ADS1015_CONF_PGA_MASK) |
                  (pga << ADS1015_CONF_PGA_OFFS);
    set_reg(ADS1015_REG_CONFIG, _config_reg);
}

void ads1015_ctrl::set_mode(bool powerdown)
{
    if (powerdown) {
        _config_reg |=  ADS1015_CONF_MODE;
    } else {
        _config_reg &= ~ADS1015_CONF_MODE;
    }
    set_reg(ADS1015_REG_CONFIG, _config_reg);
}

double ads1015_ctrl::get_value()
{
    if (!_iface)
        return nan("");

    struct coeffs {
        unsigned u;
        unsigned d;
    };

    static const coeffs gain_cfs[] = {
        {2,3},  // 6
        {1,1},  // 4
        {2,1},  // 2
        {4,1},  // 1
        {8,1},  // 0.5
        {16,1}, // 0.25
    };

    unsigned pga = (_config_reg & ADS1015_CONF_PGA_MASK) >> ADS1015_CONF_PGA_OFFS;
    if (pga > 5)
        pga = 5;

    if ((_config_reg & ADS1015_CONF_MODE) == ADS1015_CONF_MODE) {
        // Power-down mode, start conversion
        set_reg(ADS1015_REG_CONFIG, _config_reg | ADS1015_CONF_OS);

        // Wait for conversion ready
        unsigned i;
        for (i = 0; i < ADS1015_POLL_RDY_WATCHDOG; i++) {
            if (get_reg(ADS1015_REG_CONFIG) & ADS1015_CONF_OS)
                break;
        }

        if (i == ADS1015_POLL_RDY_WATCHDOG)
            return nan("");
    }

    uint16_t raw = get_reg(ADS1015_REG_CONVERSION);
    double val = (raw >> 4);
    val = val * gain_cfs[pga].d * 2 / (1000 * gain_cfs[pga].u );
    return  val;
}


uint16_t ads1015_ctrl::get_reg(ads1015_regs reg)
{
    byte_vector_t cmd = boost::assign::list_of(reg);
    _iface->write_i2c(_addr, cmd);

    byte_vector_t vec = _iface->read_i2c(_addr, 2);
    uint16_t res = (((uint16_t)vec.at(0)) << 8) | vec.at(1);
    return res;
}

void ads1015_ctrl::set_reg(ads1015_regs reg, uint16_t value)
{
    byte_vector_t cmd = boost::assign::list_of((uint8_t)reg)((uint8_t)(value >> 8))((uint8_t)(value & 0xFF));
    _iface->write_i2c(_addr, cmd);
}

