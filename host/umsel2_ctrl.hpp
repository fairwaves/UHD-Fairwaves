// Copyright 2015-2015 Fairwaves LLC
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

#ifndef INCLUDED_UMSEL2_CTRL_HPP
#define INCLUDED_UMSEL2_CTRL_HPP

#include <uhd/types/serial.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/ranges.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

#define UMSEL2_CH1_LMS_IF 360e6
#define UMSEL2_CH2_LMS_IF 400e6

/*!
 * Control UmSEL2 board.
 */
class umsel2_ctrl
{
public:
    typedef boost::shared_ptr<umsel2_ctrl> sptr;

    static sptr make(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface, const double ref_clock, const bool verbose);

    /*!
     * Query the tune range.
     * \param which values 1 or 2
     */
    virtual uhd::freq_range_t get_rx_freq_range(const int which) = 0;

    /*!
     * Tune the synthesizer
     * \param which values 1 or 2
     * \param freq the freq in Hz
     * \return the actual freq in Hz
     */
    virtual double set_rx_freq(const int which, const double freq) = 0;

    /*!
     * Query lock detect.
     * \param which values 1 or 2
     */
    virtual uhd::sensor_value_t get_locked(const int which) = 0;
};

#endif /* INCLUDED_UMSEL2_CTRL_HPP */
