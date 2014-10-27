//
// Copyright 2010 Ettus Research LLC
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

#include <uhd/utils/safe_main.hpp>
#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <iostream>

#include <uhd/types/serial.hpp>

namespace po = boost::program_options;
static const boost::uint8_t N100_EEPROM_ADDR = 0x50;

int UHD_SAFE_MAIN(int argc, char *argv[]){
    std::string args, key, val;
    unsigned dump_sz, off_sz;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args [default = \"\"]")
        ("key", po::value<std::string>(&key), "the indentifier for a value in EEPROM")
        ("val", po::value<std::string>(&val), "the new value to set, omit for readback")
        ("sz", po::value<unsigned>(&dump_sz)->default_value(256), "size of dump or erase")
        ("off", po::value<unsigned>(&off_sz)->default_value(0), "size of dump or erase")
        ("dump", "Dump EEPROM memory")
        ("erase", "Erase EEPROM memory")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help") or not (vm.count("key") or vm.count("dump") or vm.count("erase"))){
        std::cout << boost::format("USRP Burn Motherboard EEPROM %s") % desc << std::endl;
        std::cout << boost::format(
            "Omit the value argument to perform a readback,\n"
            "Or specify a new value to burn into the EEPROM.\n"
        ) << std::endl;
        return ~0;
    }

    std::cout << "Creating USRP device from address: " + args << std::endl;
    uhd::device::sptr dev = uhd::device::make(args);
    uhd::property_tree::sptr tree = dev->get_tree();
    std::cout << std::endl;

    if (vm.count("key")) {
        std::cout << "Fetching current settings from EEPROM..." << std::endl;
        uhd::usrp::mboard_eeprom_t mb_eeprom = tree->access<uhd::usrp::mboard_eeprom_t>("/mboards/0/eeprom").get();
        if (not mb_eeprom.has_key(key)){
            std::cerr << boost::format("Cannot find value for EEPROM[%s]") % key << std::endl;
            return ~0;
        }
        std::cout << boost::format("    EEPROM [\"%s\"] is \"%s\"") % key % mb_eeprom[key] << std::endl;
        std::cout << std::endl;
    }
    if (vm.count("val")){
        uhd::usrp::mboard_eeprom_t mb_eeprom; mb_eeprom[key] = val;
        std::cout << boost::format("Setting EEPROM [\"%s\"] to \"%s\"...") % key % val << std::endl;
        tree->access<uhd::usrp::mboard_eeprom_t>("/mboards/0/eeprom").set(mb_eeprom);
        std::cout << "Power-cycle the USRP device for the changes to take effect." << std::endl;
        std::cout << std::endl;
    }
    if (vm.count("dump")) {
        uhd::i2c_iface::sptr i2c;
        i2c = tree->access<uhd::i2c_iface::sptr>("/mboards/0/i2c").get();
        unsigned part_off = off_sz;
        unsigned part_sz = dump_sz;
        do {
            unsigned read_sz = (part_sz > 20) ? 20 : part_sz;

            uhd::byte_vector_t v = i2c->read_eeprom(N100_EEPROM_ADDR, part_off, read_sz);
            unsigned i;
            for (i = 0; i < v.size(); i++) {
                fprintf(stderr, "%d: %02x '%c'\n", part_off + i, v[i], (v[i] > 31 ? v[i] : '?'));
            }

            part_sz -= read_sz;
            part_off += read_sz;
        } while (part_sz > 0);
    }
    if (vm.count("erase")) {
        uhd::i2c_iface::sptr i2c;
        i2c = tree->access<uhd::i2c_iface::sptr>("/mboards/0/i2c").get();
        unsigned part_off = off_sz;
        unsigned part_sz = dump_sz;
        do {
            unsigned read_sz = (part_sz > 20) ? 20 : part_sz;

            uhd::byte_vector_t vec(read_sz);
            std::fill(vec.begin(), vec.end(), 0xFF);

            i2c->write_eeprom(N100_EEPROM_ADDR, part_off, vec);
            //uhd::byte_vector_t v = i2c->read_eeprom(N100_EEPROM_ADDR, part_off, read_sz);

            part_sz -= read_sz;
            part_off += read_sz;
        } while (part_sz > 0);
    }

    std::cout << "Done" << std::endl;
    return 0;
}
