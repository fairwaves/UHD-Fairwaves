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
#include "cores/apply_corrections.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp> //sleep
#include <boost/assign/list_of.hpp>
#include <boost/utility.hpp>

static int verbosity = 0;

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

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
    _device_ip_addr = device_addr["addr"];
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
    _ctrl = umtrx_fifo_ctrl::make(this->make_xport(UMTRX_CTRL_FRAMER, device_addr_t()), UMTRX_CTRL_SID);
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


    _tree->create<bool>(mb_path / "divsw1")
            .subscribe(boost::bind(&umtrx_impl::set_divsw1, this, _1));
    _tree->create<bool>(mb_path / "divsw2")
            .subscribe(boost::bind(&umtrx_impl::set_divsw2, this, _1));


    // TODO: Add EEPROM cell to manually override this
    _pll_div = 1;

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
        _tree->create<std::complex<double> >(tx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&tx_frontend_core_200::set_dc_offset, tx_fe, _1))
            .set(std::complex<double>(0.0, 0.0));
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
    _lms_ctrl["A"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS1, SPI_SS_AUX1, this->get_master_clock_rate() / _pll_div);
    _lms_ctrl["B"] = lms6002d_ctrl::make(_ctrl/*spi*/, SPI_SS_LMS2, SPI_SS_AUX2, this->get_master_clock_rate() / _pll_div);

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
        BOOST_FOREACH(const std::string &name, ctrl->get_tx_gains())
        {
            _tree->create<meta_range_t>(tx_rf_fe_path / "gains" / name / "range")
                .publish(boost::bind(&lms6002d_ctrl::get_tx_gain_range, ctrl, name));

            _tree->create<double>(tx_rf_fe_path / "gains" / name / "value")
                .coerce(boost::bind(&lms6002d_ctrl::set_tx_gain, ctrl, _1, name))
                .set((ctrl->get_tx_gain_range(name).start() + ctrl->get_tx_gain_range(name).stop())/2.0);
        }

        //rx freq
        _tree->create<double>(rx_rf_fe_path / "freq" / "value")
            .coerce(boost::bind(&lms6002d_ctrl::set_rx_freq, ctrl, _1));
        _tree->create<meta_range_t>(rx_rf_fe_path / "freq" / "range")
            .publish(boost::bind(&lms6002d_ctrl::get_rx_freq_range, ctrl));
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
        const std::string dc_i = _iface->mb_eeprom.get(tx_name+"-vga1-dc-i", "");
        const std::string dc_q = _iface->mb_eeprom.get(tx_name+"-vga1-dc-q", "");
        if (not dc_i.empty()) _tree->access<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_i" / "value")
            .set(boost::lexical_cast<int>(dc_i));
        if (not dc_q.empty()) _tree->access<uint8_t>(tx_rf_fe_path / "lms6002d" / "tx_dc_q" / "value")
            .set(boost::lexical_cast<int>(dc_q));
    }

    //set TCXO DAC calibration value, which is read from mboard EEPROM
    const std::string tcxo_dac = _iface->mb_eeprom.get("tcxo-dac", "");
    if (not tcxo_dac.empty()) _tree->create<uint16_t>(mb_path / "tcxo_dac" / "value")
        .subscribe(boost::bind(&umtrx_impl::set_tcxo_dac, this, _iface, _1))
        .set(boost::lexical_cast<uint16_t>(tcxo_dac));

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
}

umtrx_impl::~umtrx_impl(void)
{
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

void umtrx_impl::set_pa_dcdc_r(uint8_t val)
{
    // AD5245 control
    if (_hw_rev >= UMTRX_VER_2_3_1)
        _iface->write_i2c(BOOST_BINARY(0101100), boost::assign::list_of(0)(val));
}

void umtrx_impl::set_mb_eeprom(const uhd::i2c_iface::sptr &iface, const uhd::usrp::mboard_eeprom_t &eeprom)
{
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

uhd::sensor_value_t umtrx_impl::read_temp_c(const std::string &which)
{
    double temp = (which == "A") ? _temp_side_a.get_temp() :
                                   _temp_side_b.get_temp();
    return uhd::sensor_value_t("Temp"+which, temp, "C");
}

uhd::sensor_value_t umtrx_impl::read_pa_v(const std::string &which)
{
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
        _pa_nlow = false;
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

void umtrx_impl::set_divsw1(bool en)
{
    _iface->poke32(U2_REG_SR_ADDR(SR_DIVSW+0), (en) ? 1 : 0);
}

void umtrx_impl::set_divsw2(bool en)
{
    _iface->poke32(U2_REG_SR_ADDR(SR_DIVSW+1), (en) ? 1 : 0);
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

