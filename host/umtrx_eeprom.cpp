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

static const boost::uint8_t N100_EEPROM_ADDR = 0x50;

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
;

#if 0x18 + SERIAL_LEN + NAME_MAX_LEN >= 0xFF-7
#   error EEPROM address overlap! Get a bigger EEPROM.
#endif

void load_umtrx_eeprom(mboard_eeprom_t &mb_eeprom, i2c_iface &iface){
    //load all the N100 stuf first
    mb_eeprom = mboard_eeprom_t(iface, "N100");

    //extract the Tx VGA1 DC I/Q offset values
    {
        uint8_t val = int(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-i"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx1-vga1-dc-q"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-i"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
    {
        uint8_t val = int(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"], 1).at(0));
        mb_eeprom["tx2-vga1-dc-q"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }

    //extract the TCXO DAC calibration value
    mb_eeprom["tcxo-dac"] = uint16_bytes_to_string(
        iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"], 2)
    );

    mb_eeprom["pa_dcdc_r"] =
            boost::lexical_cast<std::string>(unsigned(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"], 1).at(0)));

    {
        uint8_t val = int(iface.read_eeprom(N100_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"], 1).at(0));
        mb_eeprom["pa_low"] = (val==255)?"":boost::lexical_cast<std::string>(int(val));
    }
}

void store_umtrx_eeprom(const mboard_eeprom_t &mb_eeprom, i2c_iface &iface){
    mb_eeprom.commit(iface, "N100");

    //store the Tx VGA1 DC I/Q offset values
    if (mb_eeprom.has_key("tx1-vga1-dc-i")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx1-vga1-dc-q")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["tx1-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx1-vga1-dc-q"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-i")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-i"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-i"]))
    );
    if (mb_eeprom.has_key("tx2-vga1-dc-q")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["tx2-vga1-dc-q"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["tx2-vga1-dc-q"]))
    );

    //extract the TCXO DAC calibration value
    if (mb_eeprom.has_key("tcxo-dac")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["tcxo-dac"],
        string_to_uint16_bytes(mb_eeprom["tcxo-dac"])
    );

    if (mb_eeprom.has_key("pa_dcdc_r")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["pa_dcdc_r"],
        byte_vector_t(1, boost::lexical_cast<unsigned>(mb_eeprom["pa_dcdc_r"]))
    );

    if (mb_eeprom.has_key("pa_low")) iface.write_eeprom(
        N100_EEPROM_ADDR, UMTRX_OFFSETS["pa_low"],
        byte_vector_t(1, boost::lexical_cast<int>(mb_eeprom["pa_low"]))
    );
}
