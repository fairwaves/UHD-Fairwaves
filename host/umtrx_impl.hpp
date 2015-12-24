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

#include "usrp2/fw_common.h"
#include "umtrx_iface.hpp"
#include "umtrx_fifo_ctrl.hpp"
#include "lms6002d_ctrl.hpp"
#include "cores/rx_frontend_core_200.hpp"
#include "cores/tx_frontend_core_200.hpp"
#include "cores/rx_dsp_core_200.hpp"
#include "cores/tx_dsp_core_200.hpp"
#include "cores/time64_core_200.hpp"
#include "ads1015_ctrl.hpp"
#include "tmp102_ctrl.hpp"
#include "power_amp.hpp"
#include "umsel2_ctrl.hpp"
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/device.hpp>
#include <uhd/utils/pimpl.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/vrt_if_packet.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <uhd/utils/tasks.hpp>


// Halfthe size of USRP2 SRAM, because we split the same SRAM into buffers for two Tx channels instead of one.
static const size_t UMTRX_SRAM_BYTES = size_t(1 << 19);
static const double UMTRX_LINK_RATE_BPS = 1000e6/8;

//framer indexes for use with make_xport()
//see .PROT_DEST() in fpga module parameters
static const size_t UMTRX_DSP_TX0_FRAMER = 0;
static const size_t UMTRX_DSP_TX1_FRAMER = 1;
static const size_t UMTRX_CTRL_FRAMER = 3;
static const size_t UMTRX_DSP_RX0_FRAMER = 4;
static const size_t UMTRX_DSP_RX1_FRAMER = 5;
static const size_t UMTRX_DSP_RX2_FRAMER = 6;
static const size_t UMTRX_DSP_RX3_FRAMER = 7;

//stream IDs used by packet dispatcher to determine destination
static const boost::uint32_t UMTRX_CTRL_SID = 1;
static const boost::uint32_t UMTRX_DSP_TX0_SID = 2;
static const boost::uint32_t UMTRX_DSP_TX1_SID = 3;

//RX stream IDs -- random -- no relevance to FPGA config
static const boost::uint32_t UMTRX_DSP_RX0_SID = 0x20;
static const boost::uint32_t UMTRX_DSP_RX1_SID = 0x21;
static const boost::uint32_t UMTRX_DSP_RX2_SID = 0x22;
static const boost::uint32_t UMTRX_DSP_RX3_SID = 0x23;

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
    typedef uhd::transport::bounded_buffer<uhd::async_metadata_t> async_md_type;
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
    bool recv_async_msg(uhd::async_metadata_t &, double);
    boost::shared_ptr<async_md_type> _old_async_queue;

private:
    enum umtrx_hw_rev {
        UMTRX_VER_2_0,
        UMTRX_VER_2_1,
        UMTRX_VER_2_2,
        UMTRX_VER_2_3_0,
        UMTRX_VER_2_3_1
    };

    enum umtrx_dcdc_ver {
        DCDC_VER_2_3_1_OLD = 0,
        DCDC_VER_2_3_1_NEW = 1,

        DCDC_VER_COUNT //Should be the last
    };

    // hardware revision
    umtrx_hw_rev _hw_rev;
    int _hw_dcdc_ver;

    unsigned _pll_div;
    const char* get_hw_rev() const;

    // Optimal VGA settings for GSM signal, according to our measurements.
    static const int UMTRX_VGA1_DEF;
    static const int UMTRX_VGA2_DEF;
    static const int UMTRX_VGA2_MIN;

    static const double _dcdc_val_to_volt[umtrx_impl::DCDC_VER_COUNT][256];

    ads1015_ctrl _sense_pwr;
    ads1015_ctrl _sense_dc;
    tmp102_ctrl  _temp_side_a;
    tmp102_ctrl  _temp_side_b;

    // Optimal device specific value
    int _umtrx_vga2_def;

    //PA control
    bool _pa_nlow;
    bool _pa_en1;
    bool _pa_en2;
    uint8_t _pa_dcdc_r;
    double _pa_power_max_dBm; // Artifical PA output power limit, dBm

    void set_pa_dcdc_r(uint8_t val);
    uint8_t get_pa_dcdc_r() const {return _pa_dcdc_r;}

    //communication interfaces
    std::string _device_ip_addr;
    umtrx_iface::sptr _iface;
    umtrx_fifo_ctrl::sptr _ctrl;
    umsel2_ctrl::sptr _umsel2;

    //controls for perifs
    uhd::dict<std::string, lms6002d_ctrl::sptr> _lms_ctrl;
    uhd::dict<std::string, uhd::power_amp::sptr> _pa;
    uhd::dict<std::string, uhd::gain_range_t> _tx_power_range; // Tx output power range

    //control for FPGA cores
    std::vector<rx_frontend_core_200::sptr> _rx_fes;
    std::vector<tx_frontend_core_200::sptr> _tx_fes;
    std::vector<rx_dsp_core_200::sptr> _rx_dsps;
    std::vector<tx_dsp_core_200::sptr> _tx_dsps;
    time64_core_200::sptr _time64;

    //helper routines
    void set_mb_eeprom(const uhd::i2c_iface::sptr &, const uhd::usrp::mboard_eeprom_t &);
    double get_master_clock_rate(void) const { return 26e6; }
    double get_master_dsp_rate(void) const { return get_master_clock_rate()/2; }
    void update_tick_rate(const double rate);
    void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &);
    void update_clock_source(const std::string &);
    void update_rx_samp_rate(const size_t, const double rate);
    void update_tx_samp_rate(const size_t, const double rate);
    void time64_self_test(void);
    void update_rates(void);
    void set_rx_fe_corrections(const std::string &mb, const std::string &board, const double);
    void set_tx_fe_corrections(const std::string &mb, const std::string &board, const double);
    void set_tcxo_dac(const umtrx_iface::sptr &, const uint16_t val);
    void detect_hw_rev(const uhd::fs_path &mb_path);
    void detect_hw_dcdc_ver(const uhd::fs_path &mb_path);
    void commit_pa_state();
    void set_enpa1(bool en);
    void set_enpa2(bool en);
    void set_nlow(bool en);
    void set_diversity(bool en, int chan);
    uhd::gain_range_t generate_tx_power_range(const std::string &which) const;
    uhd::gain_range_t generate_pa_power_range(const std::string &which) const;
    const uhd::gain_range_t &get_tx_power_range(const std::string &which) const;
    double set_tx_power(double power, const std::string &which);
    double set_pa_power(double power, const std::string &which);
    uint16_t get_tcxo_dac(const umtrx_iface::sptr &);
    uhd::transport::zero_copy_if::sptr make_xport(const size_t which, const uhd::device_addr_t &args);
    std::complex<double> get_dc_offset_correction(const std::string &which) const;
    void set_dc_offset_correction(const std::string &which, const std::complex<double> &corr);
    double set_rx_freq(const std::string &which, const double freq);
    uhd::freq_range_t get_rx_freq_range(const std::string &which) const;

    // Find a dcdc_r value to approximate requested Vout voltage
    int volt_to_dcdc_r(double v);

    static double dc_offset_int2double(uint8_t corr);
    static uint8_t dc_offset_double2int(double corr);

    //temp sensors read values in degC
    uhd::sensor_value_t read_temp_c(const std::string &which);
    uhd::sensor_value_t read_pa_v(const std::string &which);
    uhd::sensor_value_t read_dc_v(const std::string &which);
    boost::recursive_mutex _i2c_mutex;

    //status monitoring
    void status_monitor_start(const uhd::device_addr_t &device_addr);
    void status_monitor_stop(void);
    uhd::task::sptr _status_monitor_task;
    void status_monitor_handler(void);

    //tcp query server
    uhd::task::sptr _server_query_task;
    void server_query_handler(void);
    boost::asio::io_service _server_query_io_service;
    boost::shared_ptr<boost::asio::ip::tcp::acceptor> _server_query_tcp_acceptor;
    void client_query_handle(boost::shared_ptr<boost::asio::ip::tcp::socket>);
    void client_query_handle1(const boost::property_tree::ptree &request, boost::property_tree::ptree &response);

    //streaming
    std::vector<boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
    std::vector<boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;
    boost::mutex _setupMutex;
};

#endif /* INCLUDED_UMTRX_IMPL_HPP */
