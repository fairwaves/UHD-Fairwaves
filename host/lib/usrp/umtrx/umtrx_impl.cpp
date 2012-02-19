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
#include "umtrx_impl.hpp"
#include "../usrp2/fw_common.h"
#include "apply_corrections.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/udp_zero_copy.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_call.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio.hpp> //used for htonl and ntohl
#include <boost/math/special_functions/round.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <uhd/usrp/dboard_iface.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

/***********************************************************************
 * Discovery over the udp transport
 **********************************************************************/
static device_addrs_t umtrx_find(const device_addr_t &hint_) {
    return usrp2_find_generic(hint_, (char *)"umtrx", UMTRX_CTRL_ID_REQUEST, UMTRX_CTRL_ID_RESPONSE);
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr umtrx_make(const device_addr_t &device_addr) {
    return device::sptr(new umtrx_impl(device_addr));
}

UHD_STATIC_BLOCK(register_umtrx_device) {
    device::register_device(&umtrx_find, &umtrx_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
umtrx_impl::umtrx_impl(const device_addr_t &_device_addr) {
    UHD_MSG(status) << "Opening a UmTRX device..." << std::endl;
    device_addr_t device_addr = _device_addr;

    //setup the dsp transport hints (default to a large recv buff)
    if (not device_addr.has_key("recv_buff_size")) {
            //set to half-a-second of buffering at max rate
            device_addr["recv_buff_size"] = "50e6";
    }
    if (not device_addr.has_key("send_buff_size")){
        //The buffer should be the size of the SRAM on the device,
        //because we will never commit more than the SRAM can hold.
        device_addr["send_buff_size"] = boost::lexical_cast<std::string>(USRP2_SRAM_BYTES);
    }

    device_addrs_t device_args = separate_device_addr(device_addr);

    //extract the user's requested MTU size or default
    mtu_result_t user_mtu;
    user_mtu.recv_mtu = size_t(device_addr.cast<double>("recv_frame_size", udp_simple::mtu));
    user_mtu.send_mtu = size_t(device_addr.cast<double>("send_frame_size", udp_simple::mtu));

    try{
        //calculate the minimum send and recv mtu of all devices
        mtu_result_t mtu = determine_mtu(device_args[0]["addr"], user_mtu);
        for (size_t i = 1; i < device_args.size(); i++){
            mtu_result_t mtu_i = determine_mtu(device_args[i]["addr"], user_mtu);
            mtu.recv_mtu = std::min(mtu.recv_mtu, mtu_i.recv_mtu);
            mtu.send_mtu = std::min(mtu.send_mtu, mtu_i.send_mtu);
        }

        device_addr["recv_frame_size"] = boost::lexical_cast<std::string>(mtu.recv_mtu);
        device_addr["send_frame_size"] = boost::lexical_cast<std::string>(mtu.send_mtu);

        UHD_MSG(status) << boost::format("Current recv frame size: %d bytes") % mtu.recv_mtu << std::endl;
        UHD_MSG(status) << boost::format("Current send frame size: %d bytes") % mtu.send_mtu << std::endl;
    }
    catch(const uhd::not_implemented_error &){
        //just ignore this error, makes older fw work...
    }

    device_args = separate_device_addr(device_addr); //update args for new frame sizes

    ////////////////////////////////////////////////////////////////////
    // create controller objects and initialize the properties tree
    ////////////////////////////////////////////////////////////////////
    _tree = property_tree::make();
    _tree->create<std::string>("/name").set("USRP2 / N-Series Device");

    for (size_t mbi = 0; mbi < device_args.size(); mbi++) {
        const device_addr_t device_args_i = device_args[mbi];
        const std::string mb = boost::lexical_cast<std::string>(mbi);
        const std::string addr = device_args_i["addr"];
        const fs_path mb_path = "/mboards/" + mb;

        ////////////////////////////////////////////////////////////////
        // create the iface that controls i2c, spi, uart, and wb
        ////////////////////////////////////////////////////////////////
        _mbc[mb].iface = usrp2_iface::make(udp_simple::make_connected(
            addr, BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
        ));
        _tree->create<std::string>(mb_path / "name").set(_mbc[mb].iface->get_cname());
        _tree->create<std::string>(mb_path / "fw_version").set(_mbc[mb].iface->get_fw_version_string());

        _mbc[mb].dboard_iface = make_umtrx_dboard_iface(_mbc[mb].iface);

        ////////////////////////////////////////////////////////////////
        // create dboard control objects
        ////////////////////////////////////////////////////////////////

        //read the dboard eeprom to extract the dboard ids
        dboard_eeprom_t rx_db_eeprom, tx_db_eeprom, gdb_eeprom;
        rx_db_eeprom.load(*_mbc[mb].iface, USRP2_I2C_ADDR_RX_DB);
        tx_db_eeprom.load(*_mbc[mb].iface, USRP2_I2C_ADDR_TX_DB);
        gdb_eeprom.load(*_mbc[mb].iface, USRP2_I2C_ADDR_TX_DB ^ 5);

        //create a new dboard interface and manager
//        _mbc[mb].dboard_iface = make_umtrx_dboard_iface(_mbc[mb].iface);
//	if (usrp2_iface::UMTRX_REV0 == _mbc[mb].iface->get_rev()) {
//	    _mbc[mb].dboard_iface = make_lms_dboard_iface(_mbc[mb].iface);
// FIXME: UMTRX EVIL HACK for DEBUG
//	    umtrx_dboard_iface _lms_iface = _mbc[mb].dboard_iface;
	    bool rise = true;
//    printf("read LMS1=%x LMS2=%x\n", _lms_iface.read_addr(1, 0x4, rise), _lms_iface.read_addr(2, 0x4, rise));
//    printf("written LMS1=%x LMS2=%x\n", _lms_iface.write_n_check(1, 0x5, 0x32, rise), _lms_iface.write_n_check(2, 0x5, 0x32, rise));
//    printf("written LMS1=%x LMS2=%x\n", _lms_iface.write_n_check(1, 0x5, 0x3A, rise), _lms_iface.write_n_check(2, 0x5, 0x3A, rise));
	    //_lms_iface.
	    reg_dump(rise);    

/*    //TX Enable
    _lms_iface.write_addr(1, 0x05, (1<<5) | (1<<4) | (1<<3) | (1<<1));     //STXEN
    _lms_iface.write_addr(1, 0x09, 1);
    //RF Settings
//    _lms_iface.write_addr(1, 0x45, 15);     //VGA2GAIN=15
//    _lms_iface.write_addr(1, 0x42, -10-(-35)); //VGA1GAIN=-10


    _lms_iface.write_addr(1, 0x40, 0x02);
    _lms_iface.write_addr(1, 0x41, 0x19);
    _lms_iface.write_addr(1, 0x42, 0x80);
    _lms_iface.write_addr(1, 0x43, 0x80);
    _lms_iface.write_addr(1, 0x44, 0x0B);
    _lms_iface.write_addr(1, 0x45, 0x80);
    _lms_iface.write_addr(1, 0x46, 0x00);
    _lms_iface.write_addr(1, 0x47, 0x40);
    _lms_iface.write_addr(1, 0x48, 0x0C);
    _lms_iface.write_addr(1, 0x49, 0x0C);
    _lms_iface.write_addr(1, 0x4A, 0x18);
    _lms_iface.write_addr(1, 0x4B, 0x50);
    _lms_iface.write_addr(1, 0x4C, 0x00);
    _lms_iface.write_addr(1, 0x4D, 0x00);

    //TX PLL
    _lms_iface.reg_dump();
    
    //Calck NINT, NFRAC
    {
    int64_t ref_clock = 26000000;
    int64_t out_clock = 925000000;
    struct vco_sel { int64_t max; int8_t value; } freqsel[] = {
	{ 0.93*1000000000, 0x3e },
	{ 1.1425*1000000000, 0x25 },
	{ 1.3475*1000000000, 0x2d } };
    
    int i;
    int8_t found_freqsel = -1;
    for (i=0;i<(int)sizeof(freqsel)/sizeof(freqsel[0]);i++) {
	if (out_clock <= freqsel[i].max) {
	    found_freqsel=freqsel[i].value; break;
	}
    }
    
    if (found_freqsel != -1) {
        int64_t vco_x = 1 << ((found_freqsel & 0x7) - 3);
        int64_t nint = vco_x * out_clock / ref_clock;
        int64_t nfrack = (1<<23) * (vco_x * out_clock - nint * ref_clock) / ref_clock;
        
        printf("FREQSEL=%d VCO_X=%d NINT=%d  NFRACK=%d\n",
    		(int)found_freqsel, (int)vco_x, (int)nint, (int)nfrack);
    		
    	
    	_lms_iface.write_addr(1, 0x10, (nint>>1)&0xff);     //NINT[8:1]
    	_lms_iface.write_addr(1, 0x11, ((nfrack>>16)&0x7f) | ((nint&0x1)<<7));     //NINT[0]NFRACK[22:16]
    	_lms_iface.write_addr(1, 0x12, (nfrack>>8)&0xff);     //NFRACK[15:8]
    	_lms_iface.write_addr(1, 0x13, (nfrack)&0xff);     //NFRACK[7:0]
    	
    	_lms_iface.write_addr(1, 0x15, found_freqsel<<2);
    
        _lms_iface.reg_dump();
        
        //POLL VOVCO
        for (i=0;i<64;i++) {
            _lms_iface.write_addr(1, 0x19, 0x80 | i);
            usleep(50);
            int comp = _lms_iface.read_addr(1,0x1a);
            
            printf("VOVCO[%d]=%x\n", i, comp);
        }

	_lms_iface.write_addr(1, 0x19, 0x80 | 52);
    }
    
    
    }
    
    
    reg_dump();
*/
	    
//	}
//	else

            
        _tree->create<dboard_iface::sptr>(mb_path / "dboards/A/iface").set(_mbc[mb].dboard_iface);
        _mbc[mb].dboard_manager = dboard_manager::make(
            rx_db_eeprom.id, tx_db_eeprom.id, gdb_eeprom.id,
            _mbc[mb].dboard_iface, _tree->subtree(mb_path / "dboards/A")
        );

    }
}

umtrx_impl::~umtrx_impl(void){UHD_SAFE_CALL(
    BOOST_FOREACH(const std::string &mb, _mbc.keys()){
        _mbc[mb].tx_dsp->set_updates(0, 0);
    }
)}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr umtrx_impl::get_rx_stream(const uhd::stream_args_t &) {
    rx_streamer::sptr my_streamer;
    return my_streamer;
}

uhd::tx_streamer::sptr umtrx_impl::get_tx_stream(const uhd::stream_args_t &) { 
    uhd::tx_streamer::sptr FIXME; return FIXME; 
}

bool umtrx_impl::recv_async_msg(uhd::async_metadata_t &, double) { 
    return false; 
} 

/***********************************************************************
 * SPI low-level functions
 **********************************************************************/
 
// spi_config_t::EDGE_RISE is used by default
uint32_t umtrx_impl::read_addr(uint8_t lms, uint8_t addr, bool rise) {
    if(addr > 127) return 0; // incorrect address, 7 bit long expected
    if(rise) {
	BOOST_FOREACH(const std::string &mb, _mbc.keys()) {// EVIL HACK - ignore everything after 1st call
	    return _mbc[mb].iface->read_spi(lms, spi_config_t::EDGE_RISE, addr << 8, 16);
	}
    }
    BOOST_FOREACH(const std::string &mb, _mbc.keys()) {// EVIL HACK - ignore everything after 1st call
        return _mbc[mb].iface->read_spi(lms, spi_config_t::EDGE_FALL, addr << 8, 16);
    }
    return 0; // placeholder for error handling
}

uint32_t umtrx_impl::write_n_check(uint8_t lms, uint8_t addr, uint8_t data, bool rise) {
    write_addr(lms, addr, data, rise);
    return read_addr(lms, addr, rise);
}

void umtrx_impl::write_addr(uint8_t lms, uint8_t addr, uint8_t data, bool rise) {
    if(addr < 128) { // 1st bit is 1 (means 'write'), than address, than value
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        if(rise) { 
    	    BOOST_FOREACH(const std::string &mb, _mbc.keys()) {// EVIL HACK - write into all possible places
		_mbc[mb].iface->write_spi(lms, spi_config_t::EDGE_RISE, command, 16);
	    }
    	}
        else { 
    	    BOOST_FOREACH(const std::string &mb, _mbc.keys()) {// EVIL HACK - write into all possible places
    		_mbc[mb].iface->write_spi(lms, spi_config_t::EDGE_FALL, command, 16); 
    	    }
    	}
    }
}

void umtrx_impl::reg_dump(bool rise) {
    for (int i = 0; i < 128; i++) {
        printf("i=%x LMS1=%x LMS2=%x\t", i, read_addr(1, i, rise), read_addr(2, i, rise));
	if(read_addr(1, i, rise) == read_addr(2, i, rise)) printf("OK\n"); else printf("DIFF\n");
    }
}
