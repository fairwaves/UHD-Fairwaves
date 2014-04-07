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
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/bind.hpp>

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
    device::register_device(&umtrx_find, &umtrx_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
umtrx_impl::umtrx_impl(const device_addr_t &device_addr)
{
    UHD_MSG(status) << "Opening a UmTRX device..." << std::endl;

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
        device_addr["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
    ));
    _tree->create<std::string>(mb_path / "name").set(_iface->get_cname());
    _tree->create<std::string>(mb_path / "fw_version").set(_iface->get_fw_version_string());

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

    ////////////////////////////////////////////////////////////////////
    // setup the mboard eeprom
    ////////////////////////////////////////////////////////////////////
    _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
        .set(_iface->mb_eeprom)
        .subscribe(boost::bind(&umtrx_impl::set_mb_eeprom, this, _iface, _1));

    ////////////////////////////////////////////////////////////////
    // create clock control objects
    ////////////////////////////////////////////////////////////////
    _tree->create<double>(mb_path / "tick_rate")
        .publish(boost::bind(&umtrx_impl::get_master_clock_rate, this))
        .subscribe(boost::bind(&umtrx_impl::update_tick_rate, this, _1));

    ////////////////////////////////////////////////////////////////
    // reset LMS chips
    ////////////////////////////////////////////////////////////////
    {
        const boost::uint32_t clock_ctrl = _iface->peek32(U2_REG_MISC_CTRL_CLOCK);
        _iface->poke32(U2_REG_MISC_CTRL_CLOCK, clock_ctrl & ~(LMS1_RESET|LMS2_RESET));
        _iface->poke32(U2_REG_MISC_CTRL_CLOCK, clock_ctrl |  (LMS1_RESET|LMS2_RESET));
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
    // sensors on the mboard
    ////////////////////////////////////////////////////////////////
    _tree->create<sensor_value_t>(mb_path / "sensors"); //phony property so this dir exists

    ////////////////////////////////////////////////////////////////
    // create frontend control objects
    ////////////////////////////////////////////////////////////////
    _rx_fes.resize(2);
    _tx_fes.resize(2);
    _rx_fes[0] = rx_frontend_core_200::make(_iface, U2_REG_SR_ADDR(SR_RX_FRONT0));
    _rx_fes[1] = rx_frontend_core_200::make(_iface, U2_REG_SR_ADDR(SR_RX_FRONT1));
    _tx_fes[0] = tx_frontend_core_200::make(_iface, U2_REG_SR_ADDR(SR_TX_FRONT0));
    _tx_fes[1] = tx_frontend_core_200::make(_iface, U2_REG_SR_ADDR(SR_TX_FRONT1));

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

        _tree->create<std::complex<double> >(rx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&rx_frontend_core_200::set_dc_offset, rx_fe, _1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<bool>(rx_fe_path / "dc_offset" / "enable")
            .subscribe(boost::bind(&rx_frontend_core_200::set_dc_offset_auto, rx_fe, _1))
            .set(true);
        _tree->create<std::complex<double> >(rx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&rx_frontend_core_200::set_iq_balance, rx_fe, _1))
            .set(std::polar<double>(1.0, 0.0));
        _tree->create<std::complex<double> >(tx_fe_path / "dc_offset" / "value")
            .coerce(boost::bind(&tx_frontend_core_200::set_dc_offset, tx_fe, _1))
            .set(std::complex<double>(0.0, 0.0));
        _tree->create<std::complex<double> >(tx_fe_path / "iq_balance" / "value")
            .subscribe(boost::bind(&tx_frontend_core_200::set_iq_balance, tx_fe, _1))
            .set(std::polar<double>(1.0, 0.0));
    }

    ////////////////////////////////////////////////////////////////
    // create rx dsp control objects
    ////////////////////////////////////////////////////////////////
    _rx_dsps.resize(2);
    _rx_dsps[0] = rx_dsp_core_200::make(_iface, U2_REG_SR_ADDR(SR_RX_DSP0), U2_REG_SR_ADDR(SR_RX_CTRL0), UMTRX_RX_SID_BASE + 0, true);
    _rx_dsps[1] = rx_dsp_core_200::make(_iface, U2_REG_SR_ADDR(SR_RX_DSP1), U2_REG_SR_ADDR(SR_RX_CTRL1), UMTRX_RX_SID_BASE + 1, true);

    for (size_t dspno = 0; dspno < _rx_dsps.size(); dspno++){
        _rx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "tick_rate")
            .subscribe(boost::bind(&rx_dsp_core_200::set_tick_rate, _rx_dsps[dspno], _1));
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
    _tx_dsps.resize(2);
    _tx_dsps[0] = tx_dsp_core_200::make(_iface, U2_REG_SR_ADDR(SR_TX_DSP0), U2_REG_SR_ADDR(SR_TX_CTRL0), UMTRX_TX_ASYNC_SID_BASE+0);
    _tx_dsps[1] = tx_dsp_core_200::make(_iface, U2_REG_SR_ADDR(SR_TX_DSP1), U2_REG_SR_ADDR(SR_TX_CTRL1), UMTRX_TX_ASYNC_SID_BASE+1);

    for (size_t dspno = 0; dspno < _tx_dsps.size(); dspno++){
        _tx_dsps[dspno]->set_link_rate(UMTRX_LINK_RATE_BPS);
        _tree->access<double>(mb_path / "tick_rate")
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

}

umtrx_impl::~umtrx_impl(void){
    
}

void umtrx_impl::set_mb_eeprom(const uhd::i2c_iface::sptr &iface, const uhd::usrp::mboard_eeprom_t &eeprom)
{
    store_umtrx_eeprom(eeprom, *iface);
}
