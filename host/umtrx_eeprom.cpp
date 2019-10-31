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

#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/types/mac_addr.hpp>
#include <uhd/types/byte_vector.hpp>
#include <uhd/utils/byteswap.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <algorithm>
#include <iostream>
#include <cstddef>

using namespace uhd;
using namespace uhd::usrp;

static const boost::uint8_t N200_EEPROM_ADDR = 0x50;
static const size_t SERIAL_LEN = 9;
static const size_t NAME_MAX_LEN = 32 - SERIAL_LEN;

//! convert a string to a byte vector to write to eeprom
static byte_vector_t string_to_uint16_bytes(const std::string &num_str){
    const boost::uint16_t num = boost::lexical_cast<boost::uint16_t>(num_str);
    const byte_vector_t lsb_msb = boost::assign::list_of
        (boost::uint8_t(num >> 0))(boost::uint8_t(num >> 8));
    return lsb_msb;
}

//! convert a byte vector read from eeprom to a string
static std::string uint16_bytes_to_string(const byte_vector_t &bytes){
    const boost::uint16_t num = (boost::uint16_t(bytes.at(0)) << 0) | (boost::uint16_t(bytes.at(1)) << 8);
    return (num == 0 or num == 0xffff)? "" : boost::lexical_cast<std::string>(num);
}

struct n200_eeprom_map{
    uint16_t hardware;
    uint8_t mac_addr[6];
    uint32_t subnet;
    uint32_t ip_addr;
    uint16_t _pad0;
    uint16_t revision;
    uint16_t product;
    unsigned char _pad1;
    unsigned char gpsdo;
    unsigned char serial[SERIAL_LEN];
    unsigned char name[NAME_MAX_LEN];
    uint32_t gateway;
};

enum n200_gpsdo_type{
    N200_GPSDO_NONE = 0,
    N200_GPSDO_INTERNAL = 1,
    N200_GPSDO_ONBOARD = 2
};


/***********************************************************************
 * Implementation of UmTRX load/store - an extension for N100
 **********************************************************************/
static const uhd::dict<std::string, boost::uint8_t> UMTRX_OFFSETS = boost::assign::map_list_of
    // Start filling this from the end of EEPROM
    ("tx1-vga1-dc-i", 0xFF-0)  // 1 byte
    ("tx1-vga1-dc-q", 0xFF-1)  // 1 byte
    ("tcxo-dac", 0xFF-3)      // 2 bytes
    ("tx2-vga1-dc-i", 0xFF-4)  // 1 byte
    ("tx2-vga1-dc-q", 0xFF-5)  // 1 byte
    ("pa_dcdc_r", 0xFF-6)      // 1 byte
    ("pa_low",    0xFF-7)      // 1 byte
    ("pa_en1",    0xFF-8)      // 1 byte
    ("pa_en2",    0xFF-9)      // 1 byte
;

#if 0x18 + SERIAL_LEN + NAME_MAX_LEN >= 0xFF-7
#   error EEPROM address overlap! Get a bigger EEPROM.
#endif

void load_umtrx_eeprom(mboard_eeprom_t &mb_eeprom, i2c_iface &iface){
    //clear the EEPROM dict
    mb_eeprom = mboard_eeprom_t();

    ///////////////////////////////////////////////////////
    //  EEPROM values common between USRP N200 and UmTRX
    ///////////////////////////////////////////////////////

    //extract the hardware number
    mb_eeprom["hardware"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, hardware), 2)
    );

    //extract the revision number
    mb_eeprom["revision"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, revision), 2)
    );

    //extract the product code
    mb_eeprom["product"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, product), 2)
    );

    //extract the addresses
    mb_eeprom["mac-addr"] = mac_addr_t::from_bytes(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, mac_addr), 6
    )).to_string();

    boost::asio::ip::address_v4::bytes_type ip_addr_bytes;
    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, ip_addr), 4), ip_addr_bytes);
    mb_eeprom["ip-addr"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, subnet), 4), ip_addr_bytes);
    mb_eeprom["subnet"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    byte_copy(iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gateway), 4), ip_addr_bytes);
    mb_eeprom["gateway"] = boost::asio::ip::address_v4(ip_addr_bytes).to_string();

    //gpsdo capabilities
    uint8_t gpsdo_byte = iface.read_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gpsdo), 1).at(0);
    switch(n200_gpsdo_type(gpsdo_byte)){
    case N200_GPSDO_INTERNAL: mb_eeprom["gpsdo"] = "internal"; break;
    case N200_GPSDO_ONBOARD: mb_eeprom["gpsdo"] = "onboard"; break;
    default: mb_eeprom["gpsdo"] = "none";
    }

    //extract the serial
    mb_eeprom["serial"] = bytes_to_string(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, serial), SERIAL_LEN
    ));

    //extract the name
    mb_eeprom["name"] = bytes_to_string(iface.read_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, name), NAME_MAX_LEN
    ));

    //Empty serial correction: use the mac address to determine serial.
    //Older usrp2 models don't have a serial burned into EEPROM.
    //The lower mac address bits will function as the serial number.
    if (mb_eeprom["serial"].empty()){
        byte_vector_t mac_addr_bytes = mac_addr_t::from_string(mb_eeprom["mac-addr"]).to_bytes();
        unsigned serial = mac_addr_bytes.at(5) | (unsigned(mac_addr_bytes.at(4) & 0x0f) << 8);
        mb_eeprom["serial"] = std::to_string(serial);
    }

    /////////////////////////////////////////////
    //  UmTRX specific EEPROM values
    /////////////////////////////////////////////

    //extract the Tx VGA1 DC I/Q offset values
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-i"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-q"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-i"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-q"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }

    //extract the TCXO DAC calibration value
    mb_eeprom["tcxo-dac"] = uint16_bytes_to_string(
        iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"], 2)
    );

    mb_eeprom["pa_dcdc_r"] =
            boost::lexical_cast<std::string>(unsigned(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"], 1).at(0)));

    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"], 1).at(0));
        mb_eeprom["pa_low"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }

    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en1"], 1).at(0));
        mb_eeprom["pa_en1"] = (val != 0) ?"1":"0";
    }
    {
        uint8_t val = int(iface.read_eeprom(N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en2"], 1).at(0));
        mb_eeprom["pa_en2"] = (val != 0) ?"1":"0";
    }
}

void store_umtrx_eeprom(const mboard_eeprom_t &mb_eeprom, i2c_iface &iface)
{
    ///////////////////////////////////////////////////////
    //  EEPROM values common between USRP N200 and UmTRX
    ///////////////////////////////////////////////////////

    //parse the revision number
    if (mb_eeprom.has_key("hardware")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, hardware),
        string_to_uint16_bytes(mb_eeprom["hardware"])
    );

    //parse the revision number
    if (mb_eeprom.has_key("revision")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, revision),
        string_to_uint16_bytes(mb_eeprom["revision"])
    );

    //parse the product code
    if (mb_eeprom.has_key("product")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, product),
        string_to_uint16_bytes(mb_eeprom["product"])
    );

    //store the addresses
    if (mb_eeprom.has_key("mac-addr")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, mac_addr),
        mac_addr_t::from_string(mb_eeprom["mac-addr"]).to_bytes()
    );

    if (mb_eeprom.has_key("ip-addr")){
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::address_v4::from_string(mb_eeprom["ip-addr"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, ip_addr), ip_addr_bytes);
    }

    if (mb_eeprom.has_key("subnet")){
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::address_v4::from_string(mb_eeprom["subnet"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, subnet), ip_addr_bytes);
    }

    if (mb_eeprom.has_key("gateway")){
        byte_vector_t ip_addr_bytes(4);
        byte_copy(boost::asio::ip::address_v4::from_string(mb_eeprom["gateway"]).to_bytes(), ip_addr_bytes);
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gateway), ip_addr_bytes);
    }

    //gpsdo capabilities
    if (mb_eeprom.has_key("gpsdo")){
        uint8_t gpsdo_byte = N200_GPSDO_NONE;
        if (mb_eeprom["gpsdo"] == "internal") gpsdo_byte = N200_GPSDO_INTERNAL;
        if (mb_eeprom["gpsdo"] == "onboard") gpsdo_byte = N200_GPSDO_ONBOARD;
        iface.write_eeprom(N200_EEPROM_ADDR, offsetof(n200_eeprom_map, gpsdo), byte_vector_t(1, gpsdo_byte));
    }

    //store the serial
    if (mb_eeprom.has_key("serial")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, serial),
        string_to_bytes(mb_eeprom["serial"], SERIAL_LEN)
    );

    //store the name
    if (mb_eeprom.has_key("name")) iface.write_eeprom(
        N200_EEPROM_ADDR, offsetof(n200_eeprom_map, name),
        string_to_bytes(mb_eeprom["name"], NAME_MAX_LEN)
    );

    /////////////////////////////////////////////
    //  UmTRX specific EEPROM values
    /////////////////////////////////////////////

    //store the Tx VGA1 DC I/Q offset values
    if (mb_eeprom.has_key("tx1-vga1-dc-i")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx1-vga1-dc-q")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-q"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-i")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-q")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-q"]))
    );

    //extract the TCXO DAC calibration value
    if (mb_eeprom.has_key("tcxo-dac")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"],
        string_to_uint16_bytes(mb_eeprom["tcxo-dac"])
    );

    if (mb_eeprom.has_key("pa_dcdc_r")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"],
        byte_vector_t(1, boost::lexical_cast<unsigned>(mb_eeprom["pa_dcdc_r"]))
    );

    if (mb_eeprom.has_key("pa_low")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_low"]))
    );

    if (mb_eeprom.has_key("pa_en1")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en1"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_en1"]))
    );

    if (mb_eeprom.has_key("pa_en2")) iface.write_eeprom(
        N200_EEPROM_ADDR, UMTRX_OFFSETS["pa_en2"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_en2"]))
    );
}
