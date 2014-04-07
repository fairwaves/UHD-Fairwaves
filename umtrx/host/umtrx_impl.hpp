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
#include "umtrx_iface.hpp"
#include "lms6002d_ctrl.hpp"
#include "cores/rx_frontend_core_200.hpp"
#include "cores/tx_frontend_core_200.hpp"
#include "cores/rx_dsp_core_200.hpp"
#include "cores/tx_dsp_core_200.hpp"

/*
#include "time64_core_200.hpp"
*/
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/property_tree.hpp>
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
static const double UMTRX_LINK_RATE_BPS = 1000e6/8;
static const boost::uint32_t UMTRX_TX_ASYNC_SID_BASE = 2;
static const boost::uint32_t UMTRX_RX_SID_BASE = 4;

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
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:

    //communication interfaces
    umtrx_iface::sptr _iface;

    //controls for perifs
    uhd::dict<std::string, lms6002d_ctrl::sptr> _lms_ctrl;

    //control for FPGA cores
    std::vector<rx_frontend_core_200::sptr> _rx_fes;
    std::vector<tx_frontend_core_200::sptr> _tx_fes;
    std::vector<rx_dsp_core_200::sptr> _rx_dsps;
    std::vector<tx_dsp_core_200::sptr> _tx_dsps;

    //helper routines
    void set_mb_eeprom(const uhd::i2c_iface::sptr &, const uhd::usrp::mboard_eeprom_t &);
    double get_master_clock_rate(void) const { return 26e6/2; }
    void update_tick_rate(const double rate);
    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_clock_source(const std::string &);
    void update_rx_samp_rate(const size_t, const double rate);
    void update_tx_samp_rate(const size_t, const double rate);
};

#endif /* INCLUDED_UMTRX_IMPL_HPP */
