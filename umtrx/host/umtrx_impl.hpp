//
// Copyright 2012-2014 Fairwaves LLC
// Copyright 2010-2011 Ettus Research LLC
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

#ifndef INCLUDED_UMTRX_IMPL_HPP
#define INCLUDED_UMTRX_IMPL_HPP

#include "fw_common.h"
/*
#include "rx_frontend_core_200.hpp"
#include "tx_frontend_core_200.hpp"
#include "rx_dsp_core_200.hpp"
#include "tx_dsp_core_200.hpp"
#include "time64_core_200.hpp"
*/
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/gps_ctrl.hpp>
#include <uhd/device.hpp>
#include <uhd/utils/pimpl.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/asio.hpp>

// Halfthe size of USRP2 SRAM, because we split the same SRAM into buffers for two Tx channels instead of one.
static const size_t UMTRX_SRAM_BYTES = size_t(1 << 19);

//! load and store for umtrx mboard eeprom map
void load_umtrx_eeprom(uhd::usrp::mboard_eeprom_t &mb_eeprom, uhd::i2c_iface &iface);
void store_umtrx_eeprom(const uhd::usrp::mboard_eeprom_t &mb_eeprom, uhd::i2c_iface &iface);

/*!
 * UmTRX implementation guts:
 * The implementation details are encapsulated here.
 * Handles device properties and streaming...
 */
class umtrx_impl : public uhd::device {
public:
    umtrx_impl(const uhd::device_addr_t &);
    ~umtrx_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args){}
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args){}
    bool recv_async_msg(uhd::async_metadata_t &, double){}

private:

    
};

#endif /* INCLUDED_UMTRX_IMPL_HPP */
