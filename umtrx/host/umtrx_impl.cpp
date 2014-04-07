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
//        _mbc[mb].clock = umtrx_clock_ctrl::make(_mbc[mb].iface);
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

}

umtrx_impl::~umtrx_impl(void){
    
}

void umtrx_impl::set_mb_eeprom(const uhd::i2c_iface::sptr &iface, const uhd::usrp::mboard_eeprom_t &eeprom)
{
    store_umtrx_eeprom(eeprom, *iface);
}
