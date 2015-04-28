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

#ifndef INCLUDED_TMP102_CTRL_HPP
#define INCLUDED_TMP102_CTRL_HPP

#include <uhd/config.hpp>
#include <uhd/types/serial.hpp>
#include <boost/cstdint.hpp>
#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>

/**
 * @brief The tmp102_ctrl class
 *
 * Control of TI's TMP102 temperature sensors
 */
class tmp102_ctrl {
public:
        enum conversion_rate {
            TMP102_CR_025HZ = 0,
            TMP102_CR_1HZ   = 1,
            TMP102_CR_4HZ   = 2, // Default
            TMP102_CR_8HZ   = 3
        };

        enum tmp102_addr {
            TMP102_NONE   = 0,
            TMP102_GROUND = BOOST_BINARY( 1001000 ),
            TMP102_VDD    = BOOST_BINARY( 1001001 ),
            TMP102_SDA    = BOOST_BINARY( 1001010 ),
            TMP102_SCL    = BOOST_BINARY( 1001011 )
        };

        tmp102_ctrl();

        /** @brief check presence of device at @ref addr */
        static bool check(const uhd::i2c_iface::sptr& iface, tmp102_addr addr);

        void init(uhd::i2c_iface::sptr iface, tmp102_addr addr);

        void set_update_rate(conversion_rate rate);
        void set_ex_mode(bool ex_mode);
        void set_shutdown_mode(bool shutdown_mode);

        double get_temp();

private:
        enum tmp102_regs {
            REG_TEMP  = 0,
            REG_CONF  = 1,
            REG_TLOW  = 2,
            REG_THIGH = 3
        };

        enum config_reg {
            CONF_OS       = (1 << 15),
            CONF_R_OFFS   = (13),  // Read only for TMP102
            CONF_R_MASK   = (BOOST_BINARY( 11 ) << CONF_R_OFFS),
            CONF_F_OFFS   = (11),      // Fault queue
            CONF_F_MASK   = (BOOST_BINARY( 11 ) << CONF_F_OFFS),
            CONF_POL      = (1 << 10),
            CONF_TM       = (1 << 9),  // Thermostat mode
            CONF_SD       = (1 << 8),
            CONF_CR_OFFS  = (6),       // see conversion_rate
            CONF_CR_MASK  = (BOOST_BINARY( 11 ) << CONF_CR_OFFS),
            CONF_AL       = (1 << 5),
            CONF_EM       = (1 << 4)   // Extended temp mode up to +150C
        };

        enum {
            TMP102_POLL_RDY_WATCHDOG = 1000
        };

        uint16_t get_reg(tmp102_regs reg);
        void set_reg(tmp102_regs reg, uint16_t value);

        uhd::i2c_iface::sptr  _iface;
        tmp102_addr      _addr;
        uint16_t         _config;
};

#endif
