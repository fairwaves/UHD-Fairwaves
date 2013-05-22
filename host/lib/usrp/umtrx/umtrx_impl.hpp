//
// Copyright 2012-2013 Fairwaves LLC
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

// Halfthe size of USRP2 SRAM, because we split the same SRAM into buffers for two Tx channels instead of one.
static const size_t UMTRX_SRAM_BYTES = size_t(1 << 19);

/*!
 * Make a UmTRX dboard interface.
 * \param iface the UmTRX interface object
 * \return a sptr to a new dboard interface
 */
uhd::usrp::dboard_iface::sptr make_umtrx_dboard_iface(usrp2_iface::sptr iface, int lms_spi_number);

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

    // LMS-specific functions
    void reg_dump();
    void lms_write(uint8_t LMS_number, uint8_t address, uint8_t value, bool rise = true);
    uint32_t lms_read(uint8_t LMS_number, uint8_t address, bool rise = true);

private:
    uhd::property_tree::sptr _tree;
    struct mb_container_type{
        usrp2_iface::sptr iface;
        uhd::gps_ctrl::sptr gps;
        std::vector<rx_frontend_core_200::sptr> rx_fes;
        std::vector<tx_frontend_core_200::sptr> tx_fes;
        std::vector<rx_dsp_core_200::sptr> rx_dsps;
        std::vector<boost::weak_ptr<uhd::rx_streamer> > rx_streamers;
        std::vector<boost::weak_ptr<uhd::tx_streamer> > tx_streamers;
        std::vector<tx_dsp_core_200::sptr> tx_dsps;
        time64_core_200::sptr time64;
        std::vector<uhd::transport::zero_copy_if::sptr> rx_dsp_xports;
        std::vector<uhd::transport::zero_copy_if::sptr> tx_dsp_xports;
        struct db_container_type{
            uhd::usrp::dboard_iface::sptr dboard_iface;
            uhd::usrp::dboard_manager::sptr dboard_manager;
        };
        uhd::dict<std::string, db_container_type> dbc;
        size_t rx_chan_occ, tx_chan_occ;
        mb_container_type(void): rx_chan_occ(0), tx_chan_occ(0){}
    };
    uhd::dict<std::string, mb_container_type> _mbc;

    void set_mb_eeprom(const std::string &, const uhd::usrp::mboard_eeprom_t &);
//    void set_db_eeprom(const std::string &, const std::string &, const uhd::usrp::dboard_eeprom_t &);
/*
    uhd::sensor_value_t get_mimo_locked(const std::string &);
    uhd::sensor_value_t get_ref_locked(const std::string &);
*/
    void set_rx_fe_corrections(const std::string &mb, const std::string &board, const double);
    void set_tx_fe_corrections(const std::string &mb, const std::string &board, const double);
    void set_tcxo_dac(const std::string &mb, const uint16_t val);
    uint16_t get_tcxo_dac(const std::string &mb);

    double get_master_clock_rate() const { return 13e6; }

    //device properties interface
    uhd::property_tree::sptr get_tree(void) const{
        return _tree;
    }

    //io impl methods and members
    UHD_PIMPL_DECL(io_impl) _io_impl;
    void io_init(void);
    void update_tick_rate(const double rate);
    void update_rx_samp_rate(const std::string &, const size_t, const double rate);
    void update_tx_samp_rate(const std::string &, const size_t, const double rate);
    void update_rates(void);
    //update spec methods are coercers until we only accept db_name == A
    void update_rx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const std::string &, const uhd::usrp::subdev_spec_t &);
    void update_clock_source(const std::string &, const std::string &);

    //helper functions
    UHD_INLINE int fe_num_for_db(const std::string& db) { return (db == "A")?0:1; }
};

#endif /* INCLUDED_UMTRX_IMPL_HPP */
