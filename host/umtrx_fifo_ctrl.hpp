//
// Copyright 2012 Ettus Research LLC
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

#ifndef INCLUDED_UMTRX_FIFO_CTRL_HPP
#define INCLUDED_UMTRX_FIFO_CTRL_HPP

#include <uhd/types/time_spec.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/transport/zero_copy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/cstdint.hpp>
#include <uhd/types/wb_iface.hpp>
#include <string>

#include "umtrx_common.hpp"

/*!
 * The umtrx FIFO control class:
 * Provide high-speed peek/poke interface.
 */
class umtrx_fifo_ctrl : public uhd::wb_iface, public uhd::spi_iface
{
public:
    typedef UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_fifo_ctrl> sptr;

    //! Make a new FIFO control object
    static sptr make(uhd::transport::zero_copy_if::sptr xport, const boost::uint32_t sid, const size_t window_size);

    //! Set the command time that will activate
    virtual void set_time(const uhd::time_spec_t &time) = 0;

    //! Set the tick rate (converting time into ticks)
    virtual void set_tick_rate(const double rate) = 0;
};

#endif /* INCLUDED_UMTRX_FIFO_CTRL_HPP */
