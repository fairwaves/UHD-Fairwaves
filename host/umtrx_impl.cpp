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

#include "umtrx_impl.hpp"
#include "umtrx_regs.hpp"
#include "umtrx_version.hpp"
#include "umtrx_log_adapter.hpp"
#include "cores/apply_corrections.hpp"
#include <uhd/utils/log.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp> //sleep
#include <boost/assign/list_of.hpp>
#include <boost/utility.hpp>

static int verbosity = 0;

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

// Values recommended by Andrey Sviyazov
const int umtrx_impl::UMTRX_VGA1_DEF = -20;
const int umtrx_impl::UMTRX_VGA2_DEF = 22;
const int umtrx_impl::UMTRX_VGA2_MIN = 0;

const double umtrx_impl::_dcdc_val_to_volt[umtrx_impl::DCDC_VER_COUNT][256] =
{
  {
     9.38,  9.38,  9.40,  9.42,  9.42,  9.44,  9.46,  9.46,  9.48,  9.50, // 10
     9.50,  9.52,  9.54,  9.54,  9.56,  9.58,  9.58,  9.60,  9.60,  9.62, // 20
     9.64,  9.66,  9.66,  9.68,  9.70,  9.70,  9.72,  9.74,  9.76,  9.76, // 30
     9.78,  9.80,  9.82,  9.82,  9.84,  9.86,  9.88,  9.90,  9.92,  9.92, // 40
     9.94,  9.96,  9.98,  9.98, 10.00, 10.02, 10.04, 10.06, 10.06, 10.08, // 50
    10.10, 10.12, 10.14, 10.16, 10.18, 10.20, 10.20, 10.24, 10.24, 10.28, // 60
    10.30, 10.32, 10.34, 10.34, 10.36, 10.38, 10.40, 10.42, 10.44, 10.46, // 70
    10.48, 10.50, 10.52, 10.54, 10.56, 10.60, 10.62, 10.64, 10.66, 10.68, // 80
    10.70, 10.72, 10.74, 10.76, 10.78, 10.80, 10.84, 10.86, 10.88, 10.90, // 90
    10.94, 10.96, 10.98, 11.00, 11.02, 11.06, 11.06, 11.10, 11.12, 11.16, // 100
    11.18, 11.20, 11.24, 11.26, 11.28, 11.32, 11.34, 11.38, 11.40, 11.44, // 110
    11.46, 11.50, 11.50, 11.54, 11.58, 11.60, 11.64, 11.66, 11.70, 11.74, // 120
    11.76, 11.80, 11.84, 11.86, 11.90, 11.94, 11.98, 12.00, 12.02, 12.06, // 130
    12.10, 12.14, 12.18, 12.22, 12.26, 12.28, 12.32, 12.36, 12.40, 12.44, // 140
    12.48, 12.54, 12.58, 12.62, 12.64, 12.68, 12.72, 12.76, 12.82, 12.86, // 150
    12.90, 12.96, 13.00, 13.04, 13.10, 13.14, 13.20, 13.24, 13.30, 13.34, // 160
    13.38, 13.44, 13.48, 13.54, 13.60, 13.66, 13.72, 13.76, 13.82, 13.88, // 170
    13.94, 14.02, 14.06, 14.14, 14.20, 14.26, 14.30, 14.36, 14.42, 14.50, // 180
    14.56, 14.64, 14.72, 14.78, 14.86, 14.92, 15.00, 15.08, 15.16, 15.24, // 190
    15.32, 15.40, 15.46, 15.54, 15.62, 15.72, 15.80, 15.90, 16.00, 16.08, // 200
    16.18, 16.28, 16.38, 16.48, 16.58, 16.68, 16.80, 16.90, 16.96, 17.08, // 210
    17.20, 17.32, 17.44, 17.56, 17.68, 17.82, 17.94, 18.06, 18.20, 18.36, // 220
    18.48, 18.64, 18.78, 18.94, 19.02, 19.18, 19.34, 19.50, 19.68, 19.84, // 230
    20.02, 20.20, 20.38, 20.58, 20.76, 20.96, 21.18, 21.38, 21.60, 21.82, // 240
    21.92, 22.16, 22.40, 22.66, 22.92, 23.18, 23.46, 23.74, 24.02, 24.30, // 250
    24.62, 24.94, 25.28, 25.62, 25.98, 26.34
  },{
    4.84,  4.84,  4.86,  4.88,  4.88,  4.90,  4.92,  4.94,  4.94,  4.96,  // 10
    4.98,  5.00,  5.02,  5.02,  5.04,  5.06,  5.06,  5.08,  5.10,  5.12,  // 20
    5.12,  5.14,  5.16,  5.18,  5.20,  5.22,  5.22,  5.24,  5.26,  5.28,  // 30
    5.30,  5.32,  5.32,  5.34,  5.36,  5.38,  5.40,  5.42,  5.44,  5.46,  // 40
    5.48,  5.50,  5.50,  5.52,  5.54,  5.56,  5.58,  5.60,  5.62,  5.64,  // 50
    5.66,  5.68,  5.70,  5.72,  5.74,  5.76,  5.78,  5.80,  5.82,  5.86,  // 60
    5.88,  5.90,  5.92,  5.94,  5.96,  5.98,  6.00,  6.02,  6.04,  6.08,  // 70
    6.10,  6.12,  6.14,  6.16,  6.20,  6.22,  6.24,  6.28,  6.30,  6.32,  // 80
    6.34,  6.36,  6.40,  6.42,  6.44,  6.48,  6.50,  6.54,  6.56,  6.58,  // 90
    6.62,  6.64,  6.68,  6.70,  6.74,  6.76,  6.78,  6.82,  6.84,  6.88,  // 100
    6.92,  6.94,  6.98,  7.00,  7.04,  7.08,  7.12,  7.14,  7.18,  7.22,  // 110
    7.26,  7.28,  7.30,  7.34,  7.38,  7.42,  7.46,  7.50,  7.54,  7.58,  // 120
    7.62,  7.66,  7.70,  7.74,  7.78,  7.82,  7.86,  7.90,  7.92,  7.98,  // 130
    8.02,  8.06,  8.10,  8.16,  8.20,  8.26,  8.30,  8.34,  8.40,  8.44,  // 140
    8.50,  8.54,  8.60,  8.66,  8.68,  8.74,  8.78,  8.84,  8.90,  8.96,  // 150
    9.02,  9.08,  9.14,  9.20,  9.26,  9.32,  9.38,  9.44,  9.52,  9.58,  // 160
    9.62,  9.68,  9.76,  9.82,  9.90,  9.96,  10.04, 10.12, 10.18, 10.26, // 170
    10.34, 10.42, 10.50, 10.58, 10.68, 10.76, 10.80, 10.88, 10.98, 11.08, // 180
    11.16, 11.26, 11.36, 11.44, 11.54, 11.64, 11.74, 11.86, 11.96, 12.08, // 190
    12.18, 12.30, 12.36, 12.48, 12.60, 12.72, 12.84, 12.96, 13.10, 13.24, // 200
    13.36, 13.50, 13.64, 13.78, 13.94, 14.08, 14.24, 14.40, 14.48, 14.66, // 210
    14.82, 15.00, 15.18, 15.36, 15.54, 15.74, 15.92, 16.12, 16.32, 16.54, // 220
    16.76, 16.98, 17.22, 17.44, 17.58, 17.82, 18.08, 18.34, 18.62, 18.90, // 230
    19.20, 19.48, 19.80, 20.10, 20.44, 20.78, 21.12, 21.50, 21.88, 22.26, // 240
    22.48, 22.90, 23.34, 23.80, 24.26, 24.74, 25.26, 25.76, 26.32, 26.86, // 250
    27.48, 28.12, 28.78, 29.50, 29.50, 29.50
  }
};

/***********************************************************************
 * Property tree "alias" function
 **********************************************************************/

template <typename T> property<T> &property_alias(uhd::property_tree::sptr &_tree,
                                                  const uhd::fs_path &orig, const uhd::fs_path &alias)
{
    // By default route each chanel to its own antenna
    return _tree->create<T>(alias)
        .subscribe(boost::bind(&uhd::property<T>::set, boost::ref(_tree->access<T>(orig)), _1))
        .publish(boost::bind(&uhd::property<T>::get, boost::ref(_tree->access<T>(orig))));
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr umtrx_make(const device_addr_t &device_addr){
    return device::sptr(new umtrx_impl(device_addr));
}

device_addrs_t umtrx_find(const device_addr_t &hint);

UHD_STATIC_BLOCK(register_umtrx_device){
    #ifdef UHD_HAS_DEVICE_FILTER
    device::register_device(&umtrx_find, &umtrx_make, device::USRP);
    #else
    device::register_device(&umtrx_find, &umtrx_make);
    #endif
}

/***********************************************************************
 * MTU Discovery
 **********************************************************************/
struct mtu_result_t{
    size_t recv_mtu, send_mtu;
};

static std::vector<std::string> power_sensors =
        boost::assign::list_of("PR1")("PF1")("PR2")("PF2");

static std::vector<std::string> dc_sensors =
        boost::assign::list_of("zero")("Vin")("VinPA")("DCOUT");

static mtu_result_t determine_mtu(const std::string &addr, const mtu_result_t &user_mtu){
    udp_simple::sptr udp_sock = udp_simple::make_connected(
        addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    );

    //The FPGA offers 4K buffers, and the user may manually request this.
    //However, multiple simultaneous receives (2DSP slave + 2DSP master),
    //require that buffering to be used internally, and this is a safe setting.
    std::vector<boost::uint8_t> buffer(std::max(user_mtu.recv_mtu, user_mtu.send_mtu));
    usrp2_ctrl_data_t *ctrl_data = reinterpret_cast<usrp2_ctrl_data_t *>(&buffer.front());
    static const double echo_timeout = 0.020; //20 ms

    //test holler - check if its supported in this fw version
    ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
    ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
    ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
    udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));
    udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
    if (ntohl(ctrl_data->id) != USRP2_CTRL_ID_HOLLER_BACK_DUDE)
        throw uhd::not_implemented_error("holler protocol not implemented");

    size_t min_recv_mtu = sizeof(usrp2_ctrl_data_t), max_recv_mtu = user_mtu.recv_mtu;
    size_t min_send_mtu = sizeof(usrp2_ctrl_data_t), max_send_mtu = user_mtu.send_mtu;

    while (min_recv_mtu < max_recv_mtu){

        size_t test_mtu = (max_recv_mtu/2 + min_recv_mtu/2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(test_mtu);
        udp_sock->send(boost::asio::buffer(buffer, sizeof(usrp2_ctrl_data_t)));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);

        if (len >= test_mtu) min_recv_mtu = test_mtu;
        else                 max_recv_mtu = test_mtu - 4;

    }

    while (min_send_mtu < max_send_mtu){

        size_t test_mtu = (max_send_mtu/2 + min_send_mtu/2 + 3) & ~3;

        ctrl_data->id = htonl(USRP2_CTRL_ID_HOLLER_AT_ME_BRO);
        ctrl_data->proto_ver = htonl(USRP2_FW_COMPAT_NUM);
        ctrl_data->data.echo_args.len = htonl(sizeof(usrp2_ctrl_data_t));
        udp_sock->send(boost::asio::buffer(buffer, test_mtu));

        size_t len = udp_sock->recv(boost::asio::buffer(buffer), echo_timeout);
        if (len >= sizeof(usrp2_ctrl_data_t)) len = ntohl(ctrl_data->data.echo_args.len);

        if (len >= test_mtu) min_send_mtu = test_mtu;
        else                 max_send_mtu = test_mtu - 4;
    }

    mtu_result_t mtu;
    mtu.recv_mtu = min_recv_mtu;
    mtu.send_mtu = min_send_mtu;
    return mtu;
}

/***********************************************************************
 * Structors
 **********************************************************************/
umtrx_impl::umtrx_impl(const device_addr_t &device_addr)
{
    _umtrx_vga2_def = device_addr.cast<int>("lmsvga2", UMTRX_VGA2_DEF);
    _device_ip_addr = device_addr["addr"];
    UHD_MSG(status) << "UmTRX driver version: " << UMTRX_VERSION << std::endl;
    UHD_MSG(status) << "Opening a UmTRX device... " << _device_ip_addr << std::endl;

    //mtu self check -- not really doing anything with it
    mtu_result_t user_mtu;
    user_mtu.recv_mtu = size_t(device_addr.cast<double>("recv_frame_size", udp_simple::mtu));
    user_mtu.send_mtu = size_t(device_addr.cast<double>("send_frame_size", udp_simple::mtu));
    user_mtu = determine_mtu(_device_ip_addr, user_mtu);
    UHD_VAR(user_mtu.recv_mtu);
    UHD_VAR(user_mtu.send_mtu);

    ////////////////////////////////////////////////////////////////////
    // create controller objects and initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("UmTRX Device");
    const fs_path mb_path = "/mboards/0";

    ////////////////////////////////////////////////////////////////
    // create the iface that controls i2c, spi, uart, and wb
    ////////////////////////////////////////////////////////////////
    _iface = umtrx_iface::make(udp_simple::make_connected(
        _device_ip_addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    ));
    _tree->create<std::string>(mb_path / "name").set(_iface->get_cname());
    _tree->create<std::string>(mb_path / "fw_version").set(_iface->get_fw_version_string());

    //get access to the various interfaces
    _tree->create<uhd::wb_iface::sptr>(mb_path / "wb_iface").set(_iface);
    _tree->create<uhd::spi_iface::sptr>(mb_path / "spi_iface").set(_iface);
    _tree->create<uhd::i2c_iface::sptr>(mb_path / "i2c_iface").set(_iface);

    //check the fpga compatibility number
    const boost::uint32_t fpga_compat_num = _iface->peek32(U2_REG_COMPAT_NUM_RB);
    const boost::uint16_t fpga_major = fpga_compat_num >> 16, fpga_minor = fpga_compat_num & 0xffff;
    if (fpga_major != USRP2_FPGA_COMPAT_NUM){
        throw uhd::runtime_error(str(boost::format(
            "\nPlease update the firmware and FPGA images for your device.\n"
            "See the application notes for UmTRX for instructions.\n"
            "Expected FPGA compatibility number %d, but got %d:\n"
            "The FPGA build is not compatible with the host code build."
        ) % int(USRP2_FPGA_COMPAT_NUM) % fpga_major));
    }
    _tree->create<std::string>(mb_path / "fpga_version").set(str(boost::format("%u.%u") % fpga_major % fpga_minor));

    //lock the device/motherboard to this process
    _iface->lock_device(true);

    ////////////////////////////////////////////////////////////////
    // high performance settings control
    ////////////////////////////////////////////////////////////////
    _iface->poke32(U2_REG_MISC_CTRL_SFC_CLEAR, 1); //clear settings fifo control state machine
    const size_t fifo_ctrl_window(device_addr.cast<size_t>("fifo_ctrl_window", 1024)); //default gets clipped to hardware maximum
    _ctrl = umtrx_fifo_ctrl::make(this->make_xport(UMTRX_CTRL_FRAMER, device_addr_t()), UMTRX_CTRL_SID, fifo_ctrl_window);
    _ctrl->peek32(0); //test readback
    _tree->create<time_spec_t>(mb_path / "time/cmd")
        .subscribe(boost::bind(&umtrx_fifo_ctrl::set_time, _ctrl, _1));
    _tree->create<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&umtrx_fifo_ctrl::set_tick_rate, _ctrl, _1));

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(_iface->mb_eeprom)
        .subscribe(boost::bind(&umtrx_impl::set_mb_eeprom, this, _iface, _1));

    ////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////
    _tree->access<double>(mb_path / "tick_rate")
        .publish(boost::bind(&umtrx_impl::get_master_clock_rate, this))
        .subscribe(boost::bind(&umtrx_impl::update_tick_rate, this, _1));
    _tree->create<double>(mb_path / "dsp_rate")
        .publish(boost::bind(&umtrx_impl::get_master_dsp_rate, this));

    ////////////////////////////////////////////////////////////////
    // reset LMS chips
    ////////////////////////////////////////////////////////////////
    _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET);
    _iface->poke32(U2_REG_MISC_LMS_RES, 0);
    _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET);

    ////////////////////////////////////////////////////////////////////////
    // autodetect umtrx hardware rev and initialize rev. specific sensors
    ////////////////////////////////////////////////////////////////////////
    detect_hw_rev(mb_path);
    _tree->create<std::string>(mb_path / "hwrev").set(get_hw_rev());
    UHD_MSG(status) << "Detected UmTRX " << get_hw_rev() << std::endl;

    _hw_dcdc_ver = device_addr.cast<int>("dcdc_ver", -1);
    if (_hw_dcdc_ver < 0)
    {
        detect_hw_dcdc_ver(mb_path);
    } else {
        UHD_ASSERT_THROW(_hw_dcdc_ver < DCDC_VER_COUNT);
        UHD_MSG(status) << "Using DCDC version " << _hw_dcdc_ver << std::endl;
    }
    _tree->create<int>(mb_path / "hwdcdc_ver").set(_hw_dcdc_ver);

    ////////////////////////////////////////////////////////////////////////
    // setup umsel2 control when present
    ////////////////////////////////////////////////////////////////////////
    const std::string detect_umsel = device_addr.get("umsel", "off");
    if (detect_umsel != "off")
    {
        //TODO delect umsel2 automatically with I2C communication
        const bool umsel_verbose = device_addr.has_key("umsel_verbose");
        _umsel2 = umsel2_ctrl::make(_ctrl/*peek*/, _ctrl/*spi*/, this->get_master_clock_rate(), umsel_verbose);
    }

    //register lock detect for umsel2
    if (_umsel2)
    {
        _tree->create<sensor_value_t>(mb_path / "dboards" / "A" / "rx_frontends" / "0" / "sensors" / "aux_lo_locked")
            .publish(boost::bind(&umsel2_ctrl::get_locked, _umsel2, 1));
        _tree->create<sensor_value_t>(mb_path / "dboards" / "B" / "rx_frontends" / "0" / "sensors" / "aux_lo_locked")
            .publish(boost::bind(&umsel2_ctrl::get_locked, _umsel2, 2));
    }

    ////////////////////////////////////////////////////////////////////////
    // configure diversity switches
    ////////////////////////////////////////////////////////////////////////

    // note: the control is also aliased to RF frontend later
    _tree->create<bool>(mb_path / "divsw1")
            .subscribe(boost::bind(&umtrx_impl::set_diversity, this, _1, 0))
            .set(device_addr.cast<bool>("divsw1", false));
    UHD_MSG(status) << "Diversity switch for channel 1: "
                    << (_tree->access<bool>(mb_path / "divsw1").get()?"true":"false")
                    << std::endl;
    _tree->create<bool>(mb_path / "divsw2")
            .subscribe(boost::bind(&umtrx_impl::set_diversity, this, _1, 1))
            .set(device_addr.cast<bool>("divsw2", false));
    UHD_MSG(status) << "Diversity switch for channel 2: "
                    << (_tree->access<bool>(mb_path / "divsw2").get()?"true":"false")
                    << std::endl;

    ////////////////////////////////////////////////////////////////////////
    // set PLL divider
    ////////////////////////////////////////////////////////////////////////

    // TODO: Add EEPROM cell to manually override this
    _pll_div = 1;

    ////////////////////////////////////////////////////////////////////
    // get the atached PA type
    ////////////////////////////////////////////////////////////////////
    std::list<std::string> pa_types = power_amp::list_pa_str();
    std::string pa_list_str;
    BOOST_FOREACH(const std::string &pa_str, pa_types)
    {
        pa_list_str += pa_str + " ";
    }
    UHD_MSG(status) << "Known PA types: " << pa_list_str << std::endl;

    power_amp::pa_type_t pa_type = power_amp::pa_str_to_type(device_addr.cast<std::string>("pa", "NONE"));
    if (_hw_rev < UMTRX_VER_2_3_1 and pa_type != power_amp::PA_NONE)
    {
        UHD_MSG(error) << "PA type " << power_amp::pa_type_to_str(pa_type) << " is not supported for UmTRX "
                       << get_hw_rev() << ". Setting PA type to NONE." << std::endl;
        pa_type = power_amp::PA_NONE;
    }

    for (char name = 'A'; name <= 'B'; name++)
    {
        std::string name_str = std::string(1, name);
        _pa[name_str] = power_amp::make(pa_type);
        UHD_MSG(status) << "Installed PA for side" << name_str << ": " << power_amp::pa_type_to_str(pa_type) << std::endl;
    }

    if (_pa["A"])
    {
        _pa_power_max_dBm = _pa["A"]->max_power_dBm();

        double limit_w = device_addr.cast<double>("pa_power_max_w", _pa["A"]->max_power_w());
        if (limit_w != _pa["A"]->max_power_w()) {
            _pa_power_max_dBm = power_amp::w2dBm(limit_w);
        }

        double limit_dbm = device_addr.cast<double>("pa_power_max_dbm", _pa["A"]->max_power_dBm());
        if (limit_dbm != _pa["A"]->max_power_dBm()) {
            _pa_power_max_dBm = limit_dbm;
        }

        if (_pa_power_max_dBm != _pa["A"]->max_power_dBm()) {
            UHD_MSG(status) << "Limiting PA output power to: " << _pa_power_max_dBm << "dBm (" << power_amp::dBm2w(_pa_power_max_dBm) << "W)" << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////
    // create codec control objects
    ////////////////////////////////////////////////////////////////////
    for (char name = 'A'; name <= 'B'; name++)
    {
        const fs_path rx_codec_path = mb_path / ("rx_codecs") / std::string(1, name);
        _tree->create<std::string>(rx_codec_path / "name").set("RX LMS ADC");
        _tree->create<int>(rx_codec_path / "gains"); //empty cuz gains are in frontend

        const fs_path tx_codec_path = mb_path / ("tx_codecs") / std::string(1, name);
        _tree->create<std::string>(tx_codec_path / "name").set("TX LMS DAC");
        _tree->create<int>(tx_codec_path / "gains"); //empty cuz gains are in frontend
    }

    ////////////////////////////////////////////////////////////////
    // create frontend control objects
    ////////////////////////////////////////////////////////////////
    _rx_fes.resize(2);
    _tx_fes.resize(2);
    _rx_fes[0] = rx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_FRONT0));
    _rx_fes[1] = rx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_FRONT1));
    _tx_fes[0] = tx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_FRONT0));
    _tx_fes[1] = tx_frontend_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_FRONT1));

    _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        .subscribe(boost::bind(&umtrx_impl::update_rx_subdev_spec, this, _1));
    _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        .subscribe(boost::bind(&umtrx_impl::update_tx_subdev_spec, this, _1));

    for (char name = 'A'; name <= 'B'; name++)
    {
        const std::string fe_name = std::string(1, name);
        const fs_path rx_fe_path = mb_path / "rx_frontends" / fe_name;
        const fs_path tx_fe_path = mb_path / "tx_frontends" / fe_name;
        const rx_frontend_core_200::sptr rx_fe = (fe_name=="A")?_rx_fes[0]:_rx_fes[1];
        const tx_frontend_core_200::sptr tx_fe = (fe_name=="A")?_tx_fes[0]:_tx_fes[1];

        tx_fe->set_mux("IQ");
        rx_fe->set_mux(false/*no swap*/);
        _tree->create<std::complex<double> >(rx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&rx_frontend_core_200::set_dc_offset, rx_fe, _1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<bool>(rx_fe_path / "dc_offset" / "enable")
            .subscribe(boost::bind(&rx_frontend_core_200::set_dc_offset_auto, rx_fe, _1))
            .set(true);
        _tree->create<std::complex<double> >(rx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&rx_frontend_core_200::set_iq_balance, rx_fe, _1))
            .set(std::polar<double>(0.0, 0.0));
        /*
        _tree->create<std::complex<double> >(tx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&tx_frontend_core_200::set_dc_offset, tx_fe, _1))
            .set(std::complex<double>(0.0, 0.0));
        */
        _tree->create<std::complex<double> >(tx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&tx_frontend_core_200::set_iq_balance, tx_fe, _1))
            .set(std::polar<double>(0.0, 0.0));
    }

    ////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////
    _rx_dsps.resize(_iface->peek32(U2_REG_NUM_DDC));
    if (_rx_dsps.size() < 2) throw uhd::runtime_error(str(boost::format("umtrx rx_dsps %u -- (unsupported FPGA image?)") % _rx_dsps.size()));
    if (_rx_dsps.size() > 0) _rx_dsps[0] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP0), U2_REG_SR_ADDR(SR_RX_CTRL0), UMTRX_DSP_RX0_SID, true);
    if (_rx_dsps.size() > 1) _rx_dsps[1] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP1), U2_REG_SR_ADDR(SR_RX_CTRL1), UMTRX_DSP_RX1_SID, true);
    if (_rx_dsps.size() > 2) _rx_dsps[2] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP2), U2_REG_SR_ADDR(SR_RX_CTRL2), UMTRX_DSP_RX2_SID, true);
    if (_rx_dsps.size() > 3) _rx_dsps[3] = rx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_RX_DSP3), U2_REG_SR_ADDR(SR_RX_CTRL3), UMTRX_DSP_RX3_SID, true);
    _tree->create<sensor_value_t>(mb_path / "rx_dsps"); //phony property so this dir exists

    for (size_t dspno = 0; dspno < _rx_dsps.size(); dspno++){
        _rx_dsps[dspno]->set_mux("IQ", false/*no swap*/);
        _rx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "dsp_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], _1));
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_vita_rate, _rx_dsps[dspno], _1));
        fs_path rx_dsp_path = mb_path / str(boost::format("rx_dsps/%u") % dspno);
        _tree->create<meta_range_t>(rx_dsp_path / "rate/range")
            .publish(boost::bind(&rx_dsp_core_200::get_host_rates, _rx_dsps[dspno]));
        _tree->create<double>(rx_dsp_path / "rate/value")
            .set(this->get_master_clock_rate()/12) //some default
            .coerce(boost::bind(&rx_dsp_core_200::set_host_rate, _rx_dsps[dspno], _1))
            .subscribe(boost::bind(&umtrx_impl::update_rx_samp_rate, this, dspno, _1));
        _tree->create<double>(rx_dsp_path / "freq/value")
            .coerce(boost::bind(&rx_dsp_core_200::set_freq, _rx_dsps[dspno], _1));
        _tree->create<meta_range_t>(rx_dsp_path / "freq/range")
            .publish(boost::bind(&rx_dsp_core_200::get_freq_range, _rx_dsps[dspno]));
        _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
            .subscribe(boost::bind(&rx_dsp_core_200::issue_stream_command, _rx_dsps[dspno], _1));
    }

    ////////////////////////////////////////////////////////////////
    // create tx dsp control objects
    ////////////////////////////////////////////////////////////////
    _tx_dsps.resize(_iface->peek32(U2_REG_NUM_DUC));
    if (_tx_dsps.empty()) _tx_dsps.resize(1); //uhd cant support empty sides
    if (_tx_dsps.size() > 0) _tx_dsps[0] = tx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_DSP0), U2_REG_SR_ADDR(SR_TX_CTRL0), UMTRX_DSP_TX0_SID);
    if (_tx_dsps.size() > 1) _tx_dsps[1] = tx_dsp_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TX_DSP1), U2_REG_SR_ADDR(SR_TX_CTRL1), UMTRX_DSP_TX1_SID);
    _tree->create<sensor_value_t>(mb_path / "tx_dsps"); //phony property so this dir exists

    for (size_t dspno = 0; dspno < _tx_dsps.size(); dspno++){
        _tx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "dsp_rate")
            .subscribe(boost::bind(&tx_dsp_core_200::set_tick_rate, _tx_dsps[dspno], _1));
        fs_path tx_dsp_path = mb_path / str(boost::format("tx_dsps/%u") % dspno);
        _tree->create<meta_range_t>(tx_dsp_path / "rate/range")
            .publish(boost::bind(&tx_dsp_core_200::get_host_rates, _tx_dsps[dspno]));
        _tree->create<double>(tx_dsp_path / "rate/value")
            .set(this->get_master_clock_rate()/12) //some default
            .coerce(boost::bind(&tx_dsp_core_200::set_host_rate, _tx_dsps[dspno], _1))
            .subscribe(boost::bind(&umtrx_impl::update_tx_samp_rate, this, dspno, _1));
        _tree->create<double>(tx_dsp_path / "freq/value")
            .coerce(boost::bind(&tx_dsp_core_200::set_freq, _tx_dsps[dspno], _1));
        _tree->create<meta_range_t>(tx_dsp_path / "freq/range")
            .publish(boost::bind(&tx_dsp_core_200::get_freq_range, _tx_dsps[dspno]));
    }

    ////////////////////////////////////////////////////////////////
    // create time control objects
    ////////////////////////////////////////////////////////////////
    time64_core_200::readback_bases_type time64_rb_bases;
    time64_rb_bases.rb_hi_now = U2_REG_TIME64_HI_RB_IMM;
    time64_rb_bases.rb_lo_now = U2_REG_TIME64_LO_RB_IMM;
    time64_rb_bases.rb_hi_pps = U2_REG_TIME64_HI_RB_PPS;
    time64_rb_bases.rb_lo_pps = U2_REG_TIME64_LO_RB_PPS;
    _time64 = time64_core_200::make(_ctrl, U2_REG_SR_ADDR(SR_TIME64), time64_rb_bases);

    _tree->access<double>(mb_path / "tick_rate")
        .subscribe(boost::bind(&time64_core_200::set_tick_rate, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "now")
        .publish(boost::bind(&time64_core_200::get_time_now, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_now, _time64, _1));
    _tree->create<time_spec_t>(mb_path / "time" / "pps")
        .publish(boost::bind(&time64_core_200::get_time_last_pps, _time64))
        .subscribe(boost::bind(&time64_core_200::set_time_next_pps, _time64, _1));
    //setup time source props
    _tree->create<std::string>(mb_path / "time_source" / "value")
        .subscribe(boost::bind(&time64_core_200::set_time_source, _time64, _1));
    _tree->create<std::vector<std::string> >(mb_path / "time_source" / "options")
        .publish(boost::bind(&time64_core_200::get_time_sources, _time64));
    //setup reference source props
    _tree->create<std::string>(mb_path / "clock_source" / "value")
        .subscribe(boost::bind(&umtrx_impl::update_clock_source, this, _1));

    static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external");
    _tree->create<std::vector<std::string> >(mb_path / "clock_source"/ "options").set(clock_sources);

    ////////////////////////////////////////////////////////////////////
    // create RF frontend interfacing
    ////////////////////////////////////////////////////////////////////
    _lms_ctrl["A"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS1, this->get_master_clock_rate() / _pll_div);
    _lms_ctrl["B"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS2, this->get_master_clock_rate() / _pll_div);

    // LMS dboard do not have physical eeprom so we just hardcode values from host/lib/usrp/dboard/db_lms.cpp
    dboard_eeprom_t rx_db_eeprom, tx_db_eeprom, gdb_db_eeprom;
    rx_db_eeprom.id = 0xfa07;
    rx_db_eeprom.revision = _iface->mb_eeprom["revision"];
    tx_db_eeprom.id = 0xfa09;
    tx_db_eeprom.revision = _iface->mb_eeprom["revision"];

    BOOST_FOREACH(const std::string &fe_name, _lms_ctrl.keys())
    {
        lms6002d_ctrl::sptr ctrl = _lms_ctrl[fe_name];

        const fs_path rx_rf_fe_path = mb_path / "dboards" / fe_name / "rx_frontends" / "0";
        const fs_path tx_rf_fe_path = mb_path / "dboards" / fe_name / "tx_frontends" / "0";

        _tree->create<std::string>(rx_rf_fe_path / "name").set("LMS6002D");
        _tree->create<std::string>(tx_rf_fe_path / "name").set("LMS6002D");

        // Different serial numbers for each LMS on a UmTRX.
        // This is required to properly correlate calibration files to LMS chips.
        rx_db_eeprom.serial = _iface->mb_eeprom["serial"] + "." + fe_name;
        tx_db_eeprom.serial = _iface->mb_eeprom["serial"] + "." + fe_name;
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "rx_eeprom")
            .set(rx_db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "tx_eeprom")
            .set(tx_db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / fe_name / "gdb_eeprom")
            .set(gdb_db_eeprom);

        //sensors -- always say locked
        _tree->create<sensor_value_t>(rx_rf_fe_path / "sensors" / "lo_locked")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_pll_locked, ctrl));
        _tree->create<sensor_value_t>(tx_rf_fe_path / "sensors" / "lo_locked")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_pll_locked, ctrl));

        //rx gains
        BOOST_FOREACH(const std::string &name, ctrl->get_rx_gains())
        {
            _tree->create<meta_range_t>(rx_rf_fe_path / "gains" / name / "range")
                .publish(boost::bind(&lms6002d_ctrl::get_rx_gain_range, ctrl, name));

            _tree->create<double>(rx_rf_fe_path / "gains" / name / "value")
                .coerce(boost::bind(&lms6002d_ctrl::set_rx_gain, ctrl, _1, name))
                .set((ctrl->get_rx_gain_range(name).start() + ctrl->get_rx_gain_range(name).stop())/2.0);
        }

        //tx gains
        if (!_pa[fe_name])
        {
            // Use internal LMS gain control if we don't have a PA
            BOOST_FOREACH(const std::string &name, ctrl->get_tx_gains())
            {
                _tree->create<meta_range_t>(tx_rf_fe_path / "gains" / name / "range")
                    .publish(boost::bind(&lms6002d_ctrl::get_tx_gain_range, ctrl, name));

                _tree->create<double>(tx_rf_fe_path / "gains" / name / "value")
                    .coerce(boost::bind(&lms6002d_ctrl::set_tx_gain, ctrl, _1, name))
                    .set((ctrl->get_tx_gain_range(name).start() + ctrl->get_tx_gain_range(name).stop())/2.0);
            }
        } else {
            // Set LMS internal VGA1 gain to optimal value
            // VGA2 will be set in the set_tx_power()
            const int vga1 = device_addr.cast<int>("lmsvga1", UMTRX_VGA1_DEF);
            ctrl->set_tx_gain(vga1, "VGA1");
            _tx_power_range[fe_name] = generate_tx_power_range(fe_name);

            // Use PA control to control output power
            _tree->create<meta_range_t>(tx_rf_fe_path / "gains" / "PA" / "range")
                .publish(boost::bind(&umtrx_impl::get_tx_power_range, this, fe_name));

            _tree->create<double>(tx_rf_fe_path / "gains" / "PA" / "value")
                .coerce(boost::bind(&umtrx_impl::set_tx_power, this, _1, fe_name))
                // Set default output power to maximum
                .set(get_tx_power_range(fe_name).stop());

        }

        //rx freq
        _tree->create<double>(rx_rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&umtrx_impl::set_rx_freq, this, fe_name, _1));
        _tree->create<meta_range_t>(rx_rf_fe_path / "freq" / "range")
            .publish(boost::bind(&umtrx_impl::get_rx_freq_range, this, fe_name));
        _tree->create<bool>(rx_rf_fe_path / "use_lo_offset").set(false);

        //tx freq
        _tree->create<double>(tx_rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_freq, ctrl, _1));
        _tree->create<meta_range_t>(tx_rf_fe_path / "freq" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_freq_range, ctrl));
        _tree->create<bool>(tx_rf_fe_path / "use_lo_offset").set(false);

        //rx ant
        _tree->create<std::vector<std::string> >(rx_rf_fe_path / "antenna" / "options")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_antennas, ctrl));
        _tree->create<std::string>(rx_rf_fe_path / "antenna" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::set_rx_ant, ctrl, _1))
            .set("RX1");

        //tx ant
        _tree->create<std::vector<std::string> >(tx_rf_fe_path / "antenna" / "options")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_antennas, ctrl));
        _tree->create<std::string>(tx_rf_fe_path / "antenna" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::set_tx_ant, ctrl, _1))
            .set("TX2");

        //misc
        _tree->create<std::string>(rx_rf_fe_path / "connection").set("IQ");
        _tree->create<std::string>(tx_rf_fe_path / "connection").set("IQ");
        _tree->create<bool>(rx_rf_fe_path / "enabled")
            .coerce(boost::bind(&lms6002d_ctrl::set_rx_enabled, ctrl, _1));
        _tree->create<bool>(tx_rf_fe_path / "enabled")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_enabled, ctrl, _1));

        //rx bw
        _tree->create<double>(rx_rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_rx_bandwidth, ctrl, _1))
            .set(2*0.75e6);
        _tree->create<meta_range_t>(rx_rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_bw_range, ctrl));

        //tx bw
        _tree->create<double>(tx_rf_fe_path / "bandwidth" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_tx_bandwidth, ctrl, _1))
            .set(2*0.75e6);
        _tree->create<meta_range_t>(tx_rf_fe_path / "bandwidth" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_tx_bw_range, ctrl));

        //bind frontend corrections to the dboard freq props
        _tree->access<double>(tx_rf_fe_path / "freq" / "value")
            .set(0.0) //default value
            .subscribe(boost::bind(&umtrx_impl::set_tx_fe_corrections, this, "0", fe_name, _1));
        _tree->access<double>(rx_rf_fe_path / "freq" / "value")
            .set(0.0) //default value
            .subscribe(boost::bind(&umtrx_impl::set_rx_fe_corrections, this, "0", fe_name, _1));

        //tx cal props
        _tree->create<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_i" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::_set_tx_vga1dc_i_int, ctrl, _1))
            .publish(boost::bind(&lms6002d_ctrl::get_tx_vga1dc_i_int, ctrl));
        _tree->create<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_q" / "value")
            .subscribe(boost::bind(&lms6002d_ctrl::_set_tx_vga1dc_q_int, ctrl, _1))
            .publish(boost::bind(&lms6002d_ctrl::get_tx_vga1dc_q_int, ctrl));

        //set Tx DC calibration values, which are read from mboard EEPROM
        std::string tx_name = (fe_name=="A")?"tx1":"tx2";
        const std::string dc_i_str = _iface->mb_eeprom.get(tx_name+"-vga1-dc-i", "");
        const std::string dc_q_str = _iface->mb_eeprom.get(tx_name+"-vga1-dc-q", "");
        double dc_i = dc_i_str.empty() ? 0.0 : dc_offset_int2double(boost::lexical_cast<int>(dc_i_str));
        double dc_q = dc_q_str.empty() ? 0.0 : dc_offset_int2double(boost::lexical_cast<int>(dc_q_str));

        //plugin dc_offset from lms into the frontend corrections
        _tree->create<std::complex<double> >(mb_path / "tx_frontends" / fe_name / "dc_offset" / "value")
            .publish(boost::bind(&umtrx_impl::get_dc_offset_correction, this, fe_name))
            .subscribe(boost::bind(&umtrx_impl::set_dc_offset_correction, this, fe_name, _1))
            .set(std::complex<double>(dc_i, dc_q));

        //rx cal props
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_fe_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxfe_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxfe_dc_i, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_fe_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxfe_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxfe_dc_q, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_lpf_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxlpf_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxlpf_dc_i, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rx_lpf_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxlpf_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxlpf_dc_q, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2_dc_reference" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2_dc_reference, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2_dc_reference, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2a_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2a_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2a_dc_i, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2a_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2a_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2a_dc_q, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2b_dc_i" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2b_dc_i, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2b_dc_i, ctrl, _1));
        _tree->create<uint8_t>(rx_rf_fe_path / "lms6002d" / "rxvga2b_dc_q" / "value")
            .publish(boost::bind(&lms6002d_ctrl::get_rxvga2b_dc_q, ctrl))
            .subscribe(boost::bind(&lms6002d_ctrl::set_rxvga2b_dc_q, ctrl, _1));

        // Alias diversity switch control from mb_path
        property_alias<bool>(_tree, mb_path / "divsw"+(fe_name=="A"?"1":"2"), rx_rf_fe_path / "diversity");
    }

    //TCXO DAC calibration control
    _tree->create<uint16_t>(mb_path / "tcxo_dac" / "value")
        .subscribe(boost::bind(&umtrx_impl::set_tcxo_dac, this, _iface, _1));

    ////////////////////////////////////////////////////////////////////
    // post config tasks
    ////////////////////////////////////////////////////////////////////
    _tree->access<double>(mb_path / "tick_rate")
        .set(this->get_master_clock_rate());
    _tree->access<double>(mb_path / "dsp_rate")
        .set(this->get_master_dsp_rate());
    this->time64_self_test();

    //reset cordic rates and their properties to zero
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "rx_dsps"))
    {
        _tree->access<double>(mb_path / "rx_dsps" / name / "freq" / "value").set(0.0);
    }
    BOOST_FOREACH(const std::string &name, _tree->list(mb_path / "tx_dsps"))
    {
        _tree->access<double>(mb_path / "tx_dsps" / name / "freq" / "value").set(0.0);
    }

    _rx_streamers.resize(_rx_dsps.size());
    _tx_streamers.resize(_tx_dsps.size());

    subdev_spec_t rx_spec("A:0 B:0 A:0 B:0");
    rx_spec.resize(_rx_dsps.size());
    _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(rx_spec);

    subdev_spec_t tx_spec("A:0 B:0 A:0 B:0");
    tx_spec.resize(_tx_dsps.size());
    _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(tx_spec);

    _tree->access<std::string>(mb_path / "clock_source" / "value").set("internal");
    _tree->access<std::string>(mb_path / "time_source" / "value").set("none");

    //create status monitor and client handler
    this->status_monitor_start(device_addr);
}

umtrx_impl::~umtrx_impl(void)
{
    this->status_monitor_stop();

    BOOST_FOREACH(const std::string &fe_name, _lms_ctrl.keys())
    {
        lms6002d_ctrl::sptr ctrl = _lms_ctrl[fe_name];
        try
        {
            ctrl->set_rx_enabled(false);
            ctrl->set_tx_enabled(false);
        }
        catch (...){}
    }
}

int umtrx_impl::volt_to_dcdc_r(double v)
{
    if (v <= _dcdc_val_to_volt[_hw_dcdc_ver][0])
        return 0;
    else if (v >= _dcdc_val_to_volt[_hw_dcdc_ver][255])
        return 255;
    else
        return std::lower_bound(&_dcdc_val_to_volt[_hw_dcdc_ver][0], &_dcdc_val_to_volt[_hw_dcdc_ver][256], v) -
                                &_dcdc_val_to_volt[_hw_dcdc_ver][0];
}

void umtrx_impl::set_pa_dcdc_r(uint8_t val)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    // AD5245 control
    if (_hw_rev >= UMTRX_VER_2_3_1)
    {
        _pa_dcdc_r = val;
        _iface->write_i2c(BOOST_BINARY(0101100), boost::assign::list_of(0)(val));
    }
}

uhd::gain_range_t umtrx_impl::generate_tx_power_range(const std::string &which) const
{
    // Native PA range plus LMS6 VGA2 control. We keep LMS6 VGA1 constant to
    // maintain high signal quality.
    uhd::gain_range_t pa_range = generate_pa_power_range(which);
//    UHD_MSG(status) << "Original PA output power range: " << pa_range.to_pp_string() << std::endl;
    uhd::gain_range_t vga_range(pa_range.start() - (_umtrx_vga2_def-UMTRX_VGA2_MIN), pa_range.start()-1, 1.0);
    uhd::gain_range_t res_range(vga_range);
    res_range.insert(res_range.end(), pa_range.begin(), pa_range.end());
//    UHD_MSG(status) << "Generated Tx output power range: " << res_range.to_pp_string() << std::endl;
    return res_range;
}

uhd::gain_range_t umtrx_impl::generate_pa_power_range(const std::string &which) const
{
    double min_power = _pa[which]->min_power_dBm();
    double max_power = _pa_power_max_dBm;
    return uhd::gain_range_t(min_power, max_power, 0.1);
}

const uhd::gain_range_t &umtrx_impl::get_tx_power_range(const std::string &which) const
{
    return _tx_power_range[which];
}

double umtrx_impl::set_tx_power(double power, const std::string &which)
{
    double min_pa_power = _pa[which]->min_power_dBm();
    double actual_power;

    if (power >= min_pa_power)
    {
        UHD_MSG(status) << "Setting Tx power using PA (VGA2=" << _umtrx_vga2_def << ", PA=" << power << ")" << std::endl;
        // Set VGA2 to the recommended value and use PA to control Tx power
        _lms_ctrl[which]->set_tx_gain(_umtrx_vga2_def, "VGA2");
        actual_power = set_pa_power(power, which);
    } else {
        double vga2_gain = _umtrx_vga2_def - (min_pa_power-power);
        UHD_MSG(status) << "Setting Tx power using VGA2 (VGA2=" << vga2_gain << ", PA=" << min_pa_power << ")" << std::endl;
        // Set PA output power to minimum and use VGA2 to control Tx power
        actual_power = _lms_ctrl[which]->set_tx_gain(vga2_gain, "VGA2");
        actual_power = set_pa_power(min_pa_power, which) - (_umtrx_vga2_def-actual_power);
    }

    return actual_power;
}

double umtrx_impl::set_pa_power(double power, const std::string &which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    // TODO:: Use DCDC bypass for maximum output power
    // TODO:: Limit output power for UmSITE-TM3

    // Find voltage required for the requested output power
    double v = _pa[which]->dBm2v(power);
    uint8_t dcdc_val = volt_to_dcdc_r(v);
    // Set the value
    set_nlow(true);
    set_pa_dcdc_r(dcdc_val);

    // Check what power do we actually have by reading the DCDC voltage
    // and converting it to the PA power
    double v_actual = read_dc_v("DCOUT").to_real();
    double power_actual = _pa[which]->v2dBm(v_actual);

    // TODO:: Check that power is actually there by reading VSWR sensor.

    UHD_MSG(status) << "Setting PA power: Requested: " << power << "dBm = " << power_amp::dBm2w(power) << "W "
                    << "(" << v << "V dcdc_r=" << int(dcdc_val) << "). "
                    << "Actual: " << power_actual << "dBm = " << power_amp::dBm2w(power_actual) <<"W "
                    << "(" << v_actual << "V)" << std::endl;

    return power_actual;
}

void umtrx_impl::set_mb_eeprom(const uhd::i2c_iface::sptr &iface, const uhd::usrp::mboard_eeprom_t &eeprom)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    store_umtrx_eeprom(eeprom, *iface);
}

void umtrx_impl::time64_self_test(void)
{
    //check the the ticks elapsed across a sleep is within an expected range
    //this proves that the clock is the correct rate and not off by a factor

    UHD_MSG(status) << "Time register self-test... " << std::flush;
    const time_spec_t t0 = _time64->get_time_now();
    const double sleepTime = 0.50;
    boost::this_thread::sleep(boost::posix_time::milliseconds(long(sleepTime*1000)));
    const time_spec_t t1 = _time64->get_time_now();
    const double secs_elapsed = (t1 - t0).get_real_secs();
    const bool within_range = (secs_elapsed < (1.5)*sleepTime and secs_elapsed > (0.5)*sleepTime);
    UHD_MSG(status) << (within_range? "pass" : "fail") << std::endl;
}

void umtrx_impl::update_clock_source(const std::string &){}

void umtrx_impl::set_rx_fe_corrections(const std::string &mb, const std::string &board, const double lo_freq){
    apply_rx_fe_corrections(this->get_tree()->subtree("/mboards/" + mb), board, lo_freq);
}

void umtrx_impl::set_tx_fe_corrections(const std::string &mb, const std::string &board, const double lo_freq){
    apply_tx_fe_corrections(this->get_tree()->subtree("/mboards/" + mb), board, lo_freq);
}

void umtrx_impl::set_tcxo_dac(const umtrx_iface::sptr &iface, const uint16_t val){
    if (verbosity>0) printf("umtrx_impl::set_tcxo_dac(%d)\n", val);
    iface->send_zpu_action(UMTRX_ZPU_REQUEST_SET_VCTCXO_DAC, val);
}

uint16_t umtrx_impl::get_tcxo_dac(const umtrx_iface::sptr &iface){
    uint16_t val = iface->send_zpu_action(UMTRX_ZPU_REQUEST_GET_VCTCXO_DAC, 0);
    if (verbosity>0) printf("umtrx_impl::get_tcxo_dac(): %d\n", val);
    return (uint16_t)val;
}

std::complex<double> umtrx_impl::get_dc_offset_correction(const std::string &which) const
{
    return std::complex<double>(
        dc_offset_int2double(_lms_ctrl[which]->get_tx_vga1dc_i_int()),
        dc_offset_int2double(_lms_ctrl[which]->get_tx_vga1dc_q_int()));
}

void umtrx_impl::set_dc_offset_correction(const std::string &which, const std::complex<double> &corr)
{
    _lms_ctrl[which]->_set_tx_vga1dc_i_int(dc_offset_double2int(corr.real()));
    _lms_ctrl[which]->_set_tx_vga1dc_q_int(dc_offset_double2int(corr.imag()));
}

double umtrx_impl::dc_offset_int2double(uint8_t corr)
{
    return (corr-128)/128.0;
}

uint8_t umtrx_impl::dc_offset_double2int(double corr)
{
    return (int)(corr*128 + 128.5);
}

double umtrx_impl::set_rx_freq(const std::string &which, const double freq)
{
    if (_umsel2)
    {
        const double target_lms_freq = (which=="A")?UMSEL2_CH1_LMS_IF:UMSEL2_CH2_LMS_IF;
        const double actual_lms_freq = _lms_ctrl[which]->set_rx_freq(target_lms_freq);

        const double target_umsel_freq = freq - actual_lms_freq;
        const double actual_umsel_freq = _umsel2->set_rx_freq((which=="A")?1:2, target_umsel_freq);

        /*
        std::cout << "target_total_freq " << freq/1e6 << " MHz" << std::endl;
        std::cout << "target_lms_freq " << target_lms_freq/1e6 << " MHz" << std::endl;
        std::cout << "actual_lms_freq " << actual_lms_freq/1e6 << " MHz" << std::endl;
        std::cout << "target_umsel_freq " << target_umsel_freq/1e6 << " MHz" << std::endl;
        std::cout << "actual_umsel_freq " << actual_umsel_freq/1e6 << " MHz" << std::endl;
        std::cout << "actual_total_freq " << (actual_umsel_freq + actual_lms_freq)/1e6 << " MHz" << std::endl;
        //*/

        return actual_umsel_freq + actual_lms_freq;
    }
    else
    {
        return _lms_ctrl[which]->set_rx_freq(freq);
    }
}

uhd::freq_range_t umtrx_impl::get_rx_freq_range(const std::string &which) const
{
    if (_umsel2)
    {
        const double target_lms_freq = (which=="A")?UMSEL2_CH1_LMS_IF:UMSEL2_CH2_LMS_IF;
        const uhd::freq_range_t range_umsel = _umsel2->get_rx_freq_range((which=="A")?1:2);
        return uhd::freq_range_t(
            range_umsel.start()+target_lms_freq,
            range_umsel.stop()+target_lms_freq);
    }
    else
    {
        return _lms_ctrl[which]->get_rx_freq_range();
    }
}

uhd::sensor_value_t umtrx_impl::read_temp_c(const std::string &which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    double temp = (which == "A") ? _temp_side_a.get_temp() :
                                   _temp_side_b.get_temp();
    return uhd::sensor_value_t("Temp"+which, temp, "C");
}

uhd::sensor_value_t umtrx_impl::read_pa_v(const std::string &which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    unsigned i;
    for (i = 0; i < 4; i++) {
        if (which == power_sensors[i])
            break;
    }
    UHD_ASSERT_THROW(i < 4);

    _sense_pwr.set_input((ads1015_ctrl::ads1015_input)
                         (ads1015_ctrl::ADS1015_CONF_AIN0_GND + i));
    double val = _sense_pwr.get_value() * 10;
    return uhd::sensor_value_t("Voltage"+which, val, "V");
}

uhd::sensor_value_t umtrx_impl::read_dc_v(const std::string &which)
{
    boost::recursive_mutex::scoped_lock l(_i2c_mutex);
    unsigned i;
    for (i = 0; i < 4; i++) {
        if (which == dc_sensors[i])
            break;
    }
    UHD_ASSERT_THROW(i < 4);

    _sense_dc.set_input((ads1015_ctrl::ads1015_input)
                         (ads1015_ctrl::ADS1015_CONF_AIN0_GND + i));
    double val = _sense_dc.get_value() * 40;
    return uhd::sensor_value_t("Voltage"+which, val, "V");
}

void umtrx_impl::detect_hw_rev(const fs_path& mb_path)
{
    //UmTRX v2.0 doesn't have temp sensors
    //UmTRX v2.1 has a temp sensor only on A side
    //UmTRX v2.2 has both temp sensor
    //UmTRX v2.3.0 has power sensors ADC
    //UmTRX v2.3.1 has power supply ADC & programmed resistor array

    if (!tmp102_ctrl::check(_iface, tmp102_ctrl::TMP102_SDA)) {
        _tree->create<sensor_value_t>(mb_path / "sensors"); //phony property so this dir exists
        _hw_rev = UMTRX_VER_2_0;
        return;
    }
    // Initialize side A temp sensor
    _temp_side_a.init(_iface, tmp102_ctrl::TMP102_SDA);
    _temp_side_a.set_ex_mode(true);
    _tree->create<sensor_value_t>(mb_path / "sensors" / "tempA")
        .publish(boost::bind(&umtrx_impl::read_temp_c, this, "A"));
    UHD_MSG(status) << this->read_temp_c("A").to_pp_string() << std::endl;

    if (!tmp102_ctrl::check(_iface, tmp102_ctrl::TMP102_SCL)) {
        _hw_rev = UMTRX_VER_2_1;
        return;
    }
    // Initialize side B temp sensor
    _temp_side_b.init(_iface, tmp102_ctrl::TMP102_SCL);
    _temp_side_b.set_ex_mode(true);
    _tree->create<sensor_value_t>(mb_path / "sensors" / "tempB")
        .publish(boost::bind(&umtrx_impl::read_temp_c, this, "B"));
    UHD_MSG(status) << this->read_temp_c("B").to_pp_string() << std::endl;

    if (!ads1015_ctrl::check(_iface, ads1015_ctrl::ADS1015_ADDR_VDD)) {
        _hw_rev = UMTRX_VER_2_2;
        return;
    }
    //Initialize PA sense ADC
    _sense_pwr.init(_iface, ads1015_ctrl::ADS1015_ADDR_VDD);
    _sense_pwr.set_mode(true);
    _sense_pwr.set_pga(ads1015_ctrl::ADS1015_PGA_2_048V);
    for (unsigned i = 0; i < power_sensors.size(); i++) {
        _tree->create<sensor_value_t>(mb_path / "sensors" / "voltage"+power_sensors[i])
            .publish(boost::bind(&umtrx_impl::read_pa_v, this, power_sensors[i]));
        UHD_MSG(status) << this->read_pa_v(power_sensors[i]).to_pp_string() << std::endl;
    }

    if (!ads1015_ctrl::check(_iface, ads1015_ctrl::ADS1015_ADDR_GROUND)) {
        _hw_rev = UMTRX_VER_2_3_0;
        return;
    }
    _sense_dc.init(_iface, ads1015_ctrl::ADS1015_ADDR_GROUND);
    _sense_dc.set_mode(true);
    _sense_dc.set_pga(ads1015_ctrl::ADS1015_PGA_1_024V);
    for (unsigned i = 0; i < power_sensors.size(); i++) {
        _tree->create<sensor_value_t>(mb_path / "sensors" / "voltage"+dc_sensors[i])
            .publish(boost::bind(&umtrx_impl::read_dc_v, this, dc_sensors[i]));
        UHD_MSG(status) << this->read_dc_v(dc_sensors[i]).to_pp_string() << std::endl;
    }

    _hw_rev = UMTRX_VER_2_3_1;
    _tree->create<uint8_t>(mb_path / "pa_dcdc_r")
            .subscribe(boost::bind(&umtrx_impl::set_pa_dcdc_r, this, _1));

    std::string pa_dcdc_r = _iface->mb_eeprom.get("pa_dcdc_r", "");
    char* pa_dcdc_r_env = getenv("UMTRX_PA_DCDC_R");
    if (pa_dcdc_r_env) {
        UHD_MSG(status) << "EEPROM value of pa_dcdc_r:" << pa_dcdc_r.c_str()
                        << " is overriden with env UMTRX_PA_DCDC_R:"
                        << pa_dcdc_r_env << std::endl;
        pa_dcdc_r = pa_dcdc_r_env;
    }
    if (pa_dcdc_r.empty())
        set_pa_dcdc_r(0);
    else
        set_pa_dcdc_r(boost::lexical_cast<unsigned>(pa_dcdc_r));

    _pa_en1 = (boost::lexical_cast<int>(_iface->mb_eeprom.get("pa_en1", "1")) == 1);
    _pa_en2 = (boost::lexical_cast<int>(_iface->mb_eeprom.get("pa_en2", "1")) == 1);

    if (getenv("UMTRX_PA_EN1")) _pa_en1 = (boost::lexical_cast<int>(getenv("UMTRX_PA_EN1")) != 0);
    if (getenv("UMTRX_PA_EN2")) _pa_en2 = (boost::lexical_cast<int>(getenv("UMTRX_PA_EN2")) != 0);

    std::string pa_low = _iface->mb_eeprom.get("pa_low", "");
    char* pa_low_env = getenv("UMTRX_PA_LOW");
    if (pa_low_env) {
        UHD_MSG(status) << "EEPROM value of pa_low:" << pa_low.c_str()
                        << " is overriden with env UMTRX_PA_LOW:"
                        << pa_low_env << std::endl;
        pa_low = pa_low_env;
    }
    if (pa_low.empty())
        _pa_nlow = true; //Turn off Vin bypass by default
    else
        _pa_nlow = (boost::lexical_cast<int>(pa_low) == 0);

    _tree->create<bool>(mb_path / "pa_en1")
            .subscribe(boost::bind(&umtrx_impl::set_enpa1, this, _1));
    _tree->create<bool>(mb_path / "pa_en2")
            .subscribe(boost::bind(&umtrx_impl::set_enpa2, this, _1));
    _tree->create<bool>(mb_path / "pa_nlow")
            .subscribe(boost::bind(&umtrx_impl::set_nlow, this, _1));

    commit_pa_state();
    UHD_MSG(status) << "PA low=`" << pa_low.c_str()
                    << "` PA dcdc_r=`" << pa_dcdc_r.c_str()
                    << "`" << std::endl;
}

void umtrx_impl::commit_pa_state()
{
    if (_hw_rev >= UMTRX_VER_2_3_1)
        _iface->poke32(U2_REG_MISC_LMS_RES, LMS1_RESET | LMS2_RESET
                   | PAREG_ENDCSYNC
                   | ((_pa_nlow) ? PAREG_NLOW_PA : 0)
                   | ((_pa_en1)  ? PAREG_ENPA1 : 0)
                   | ((_pa_en2)  ? PAREG_ENPA2 : 0));
}

void umtrx_impl::set_enpa1(bool en)
{
    _pa_en1 = en; commit_pa_state();
}

void umtrx_impl::set_enpa2(bool en)
{
    _pa_en2 = en; commit_pa_state();
}

void umtrx_impl::set_nlow(bool en)
{
    _pa_nlow = en; commit_pa_state();
}

void umtrx_impl::set_diversity(bool en, int chan)
{
    // chan 0 has inversed switch polarity
    // chan 1 has straight switch polarity
    _iface->poke32(U2_REG_SR_ADDR(SR_DIVSW+chan), (en != (chan==1)) ? 0 : 1);
}

const char* umtrx_impl::get_hw_rev() const
{
    switch (_hw_rev) {
    case UMTRX_VER_2_0:    return "2.0";
    case UMTRX_VER_2_1:    return "2.1";
    case UMTRX_VER_2_2:    return "2.2";
    case UMTRX_VER_2_3_0:  return "2.3.0";
    case UMTRX_VER_2_3_1:  return "2.3.1";
    default:               return "[unknown]";
    }
}

void umtrx_impl::detect_hw_dcdc_ver(const uhd::fs_path &)
{
    _hw_dcdc_ver = DCDC_VER_2_3_1_OLD;
    if (_hw_rev < UMTRX_VER_2_3_1)
    {
        return;
    }

    uint8_t old = _pa_dcdc_r;
    bool old_pa_nlow = _pa_nlow;

    set_nlow(true);
    set_pa_dcdc_r(0);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    double v_actual = read_dc_v("DCOUT").to_real();

    double err_min = std::abs(v_actual - _dcdc_val_to_volt[0][0]);
    for (unsigned j = 1; j < DCDC_VER_COUNT; ++j) {
       double err = std::abs(v_actual - _dcdc_val_to_volt[j][0]);
       if (err < err_min) {
           err_min = err;
           _hw_dcdc_ver = j;
       }
    }
    set_pa_dcdc_r(old);
    set_nlow(old_pa_nlow);

    UHD_MSG(status) << "Detected UmTRX DCDC ver. " << _hw_dcdc_ver
                    << " (err: " << err_min << ")" << std::endl;
}
