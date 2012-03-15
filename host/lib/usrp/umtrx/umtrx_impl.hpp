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

#ifndef INCLUDED_UMTRX_IMPL_HPP
#define INCLUDED_UMTRX_IMPL_HPP

#include "../usrp2/fw_common.h"
#include "../usrp2/usrp2_iface.hpp"
#include "../usrp2/usrp2_impl.hpp"
#include "rx_frontend_core_200.hpp"
#include "tx_frontend_core_200.hpp"
#include "rx_dsp_core_200.hpp"
#include "tx_dsp_core_200.hpp"
#include "time64_core_200.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
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
/*!
 * Make a UmTRX dboard interface.
 * \param iface the UmTRX interface object
 * \return a sptr to a new dboard interface
 */
uhd::usrp::dboard_iface::sptr make_umtrx_dboard_iface(usrp2_iface::sptr iface);

/*!
 * UmTRX implementation guts:
 * The implementation details are encapsulated here.
 * Handles device properties and streaming...
 */
class umtrx_impl : public uhd::device {
    uhd::property_tree::sptr _tree;
    struct mb_container_type {
        usrp2_iface::sptr iface;
        uhd::gps_ctrl::sptr gps;
        rx_frontend_core_200::sptr rx_fe;
        tx_frontend_core_200::sptr tx_fe;
        std::vector<rx_dsp_core_200::sptr> rx_dsps;
        std::vector<boost::weak_ptr<uhd::rx_streamer> > rx_streamers;
        std::vector<boost::weak_ptr<uhd::tx_streamer> > tx_streamers;
        tx_dsp_core_200::sptr tx_dsp;
        time64_core_200::sptr time64;
        std::vector<uhd::transport::zero_copy_if::sptr> rx_dsp_xports;
        uhd::transport::zero_copy_if::sptr tx_dsp_xport;
        uhd::usrp::dboard_manager::sptr dboard_manager;
        uhd::usrp::dboard_iface::sptr dboard_iface;
        size_t rx_chan_occ, tx_chan_occ;
        mb_container_type(void): rx_chan_occ(0), tx_chan_occ(0){}
    };
    uhd::dict<std::string, mb_container_type> _mbc;

    void set_mb_eeprom(const std::string &, const uhd::usrp::mboard_eeprom_t &);
    void set_db_eeprom(const std::string &mb, const std::string &type, const uhd::usrp::dboard_eeprom_t &db_eeprom);
/*
    uhd::sensor_value_t get_mimo_locked(const std::string &);
    uhd::sensor_value_t get_ref_locked(const std::string &);

    void set_rx_fe_corrections(const std::string &mb, const double);
    void set_tx_fe_corrections(const std::string &mb, const double);
*/
    //device properties interface
    uhd::property_tree::sptr get_tree(void) const { return _tree; }

    //update spec methods are coercers until we only accept db_name == A
    void update_rx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);

public:
    umtrx_impl(const uhd::device_addr_t &);
    ~umtrx_impl(void);

    //the IO interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
    bool recv_async_msg(uhd::async_metadata_t &, double);

// LMS-specific functions
    void reg_dump(bool rise = true);
    void write_addr(uint8_t LMS_number, uint8_t address, uint8_t value, bool rise = true);
    uint32_t read_addr(uint8_t LMS_number, uint8_t address, bool rise = true);
    uint32_t write_n_check(uint8_t LMS_number, uint8_t address, uint8_t value, bool rise = true);
};

#endif 
