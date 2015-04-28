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

#ifndef INCLUDED_ADS1015_CTRL_HPP
#define INCLUDED_ADS1015_CTRL_HPP

#include <uhd/config.hpp>
#include <uhd/types/serial.hpp>
#include <boost/cstdint.hpp>
#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>

/**
 * @brief The ads1015_ctrl class
 *
 * Control of TI's ADS1015 12bit ADC with 4 muliplexers
 */
class ads1015_ctrl {
public:
    enum ads1015_addr {
        ADS1015_NONE        = 0,
        ADS1015_ADDR_GROUND = BOOST_BINARY( 1001000 ),
        ADS1015_ADDR_VDD    = BOOST_BINARY( 1001001 ),
        ADS1015_ADDR_SDA    = BOOST_BINARY( 1001010 ),
        ADS1015_ADDR_SCL    = BOOST_BINARY( 1001011 )
    };

    enum ads1015_input {
        ADS1015_CONF_AIN0_AIN1 = 0,
        ADS1015_CONF_AIN0_AIN3 = 1,
        ADS1015_CONF_AIN1_AIN1 = 2,
        ADS1015_CONF_AIN2_AIN1 = 3,

        ADS1015_CONF_AIN0_GND = 4,
        ADS1015_CONF_AIN1_GND = 5,
        ADS1015_CONF_AIN2_GND = 6,
        ADS1015_CONF_AIN3_GND = 7,
    };

    enum ads1015_pga {
        ADS1015_PGA_6_144V = 0,
        ADS1015_PGA_4_096V = 1,
        ADS1015_PGA_2_048V = 2,
        ADS1015_PGA_1_024V = 3,
        ADS1015_PGA_0_512V = 4,
        ADS1015_PGA_0_256V = 5,
    };

    enum ads1015_rate {
        ADS1015_RATE_128SPS  = 0,
        ADS1015_RATE_250SPS  = 1,
        ADS1015_RATE_490SPS  = 2,
        ADS1015_RATE_920SPS  = 3,
        ADS1015_RATE_1600SPS = 4,
        ADS1015_RATE_2400SPS = 5,
        ADS1015_RATE_3300SPS = 6,
    };

    enum ads1015_comp {
        ADS1015_COMP_QUE_1   = 0,
        ADS1015_COMP_QUE_2   = 1,
        ADS1015_COMP_QUE_4   = 2,
        ADS1015_COMP_QUE_DIS = 3,
    };

    ads1015_ctrl();

    void init(uhd::i2c_iface::sptr iface, ads1015_addr addr);

    /** @brief check presence of device at @ref addr */
    static bool check(const uhd::i2c_iface::sptr& iface, ads1015_addr addr);

    void set_input(ads1015_input input);
    void set_pga(ads1015_pga input);

    double get_value();
    void set_mode(bool powerdown);

private:
    enum ads1015_regs {
        ADS1015_REG_CONVERSION = 0,
        ADS1015_REG_CONFIG     = 1,
        ADS1015_REG_LO_THRES   = 2,
        ADS1015_REG_HI_THRES   = 3
    };

    enum ads1015_config {
        ADS1015_CONF_OS       = (1 << 15),
        ADS1015_CONF_MUX_OFFS = (12),
        ADS1015_CONF_MUX_MASK = (BOOST_BINARY( 111 ) << ADS1015_CONF_MUX_OFFS),
        ADS1015_CONF_PGA_OFFS = (9),
        ADS1015_CONF_PGA_MASK = (BOOST_BINARY( 111 ) << ADS1015_CONF_PGA_OFFS),
        ADS1015_CONF_MODE     = (1 << 8),  ///< 1 - Power-down single conversion mode
        ADS1015_CONF_DR_OFFS  = (5),
        ADS1015_CONF_DR_MASK  = (BOOST_BINARY( 111 ) << ADS1015_CONF_DR_OFFS),
        ADS1015_CONF_COMP_MODE= (1 << 4),  ///< 1 - Window comparator
        ADS1015_CONF_COMP_POL = (1 << 3),  ///< 1 - Active high
        ADS1015_CONF_COMP_LAT = (1 << 2),  ///< 1 - Latching compartator
        ADS1015_COMP_QUE_OFFS = (0),
        ADS1015_CONF_QUE_MASK = (BOOST_BINARY( 11 ) << ADS1015_COMP_QUE_OFFS),
    };

    enum {
        ADS1015_POLL_RDY_WATCHDOG = 1000
    };

    uint16_t get_reg(ads1015_regs reg);
    void set_reg(ads1015_regs reg, uint16_t value);

    uhd::i2c_iface::sptr   _iface;
    ads1015_addr           _addr;
    unsigned               _config_reg;
};



#endif

