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
#include <uhd/utils/safe_main.hpp>
#include <uhd/device.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/property_tree.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace std;

int UHD_SAFE_MAIN(int argc, char **argv) {
    string args;
    unsigned lms;
    po::options_description desc("available options");
    desc.add_options()
        ("help", "usage instructions")
        ("args", po::value<string>(&args)->default_value(""), "device address args [default = \"\"]")
        ("address", po::value<unsigned>(), "the address for a register (decimal)")
        ("data", po::value<unsigned>(), "the new value to be written to register (decimal), omit for reading")
        ("lms", po::value<unsigned>(&lms)->default_value(1), "the LMS to be used (decimal), defaults to 1")
        ("fall", "use FALL signal edge for SPI, defaults to RISE");
// N. B: using po::value<uint8_t> causes boost to crap and ignore correct option value
// I miss GNU/gengetopt so much...
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") or not vm.count("address")) {
        cerr << boost::format("LMS register dumper, %s\n") % desc ;
        cerr << "Omit the data argument to perform a readback,\nOr specify a new data to write into the address.\n";
        return 1;
    }

    if (vm["address"].as<unsigned>() > 127) {
	cerr << "Expected register address is [0; 127], received " << vm["address"].as<unsigned>() << "\n";
	return 2;
    }
    
    uint8_t address = vm["address"].as<unsigned>();

    if ((1 != lms) && (2 != lms)) {
	cerr << "Unexpected LMS number supplied: " << lms << ", only 1 & 2 are available.\n";
	return 3;
    } 

// establish SPI configuration
    uhd::spi_config_t front = vm.count("fall")?(uhd::spi_config_t::EDGE_FALL):(uhd::spi_config_t::EDGE_RISE);
    uhd::usrp::dboard_iface::unit_t lms_unit = (uhd::usrp::dboard_iface::unit_t)lms;
    cerr << boost::format("Using %s SPI on LMS unit ") % (vm.count("fall")?("EDGE_FALL"):("EDGE_RISE")) << lms_unit;
    cerr << "\nCreating UmTRX device from address: " << args << "\n";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();

    const uhd::fs_path mb_path = "/mboards/0";
    const string mb_name = tree->access<std::string>(mb_path / "name").get();
    if (mb_name.find("UMTRX") != string::npos or mb_name.find("UmTRX") != string::npos) {
	cerr << "UmTRX detected.\n";
    } else {
        cerr << "No supported hardware found.\n";
        return 4;
    }

    uhd::usrp::dboard_iface::sptr dbif = usrp->get_tx_dboard_iface(0);

    if (vm.count("data")) {
	if (vm["data"].as<unsigned>() > 255) {
	    cerr << "Expected data for register is [0; 127], received " << vm["data"].as<unsigned>() << "\n";
	    return 5;
	}
	uint8_t data = vm["data"].as<unsigned>();
        cerr << "Writing " << hex << data << " to register " << hex << address << "... ";
//	cerr << dbif->read_write_spi(lms_unit, front, (((uint16_t)0x80 | (uint16_t)address) << 8) | (uint16_t)data, 16);
	cerr << dbif->read_write_spi(uhd::usrp::dboard_iface::UNIT_TX, front, (((uint16_t)0x80 | (uint16_t)address) << 8) | (uint16_t)data, 16);
    } else {
	cerr << "Reading register " << hex << address << "... ";
//	cerr << dbif->read_write_spi(lms_unit, front, address << 8, 16);
	cerr << dbif->read_write_spi(uhd::usrp::dboard_iface::UNIT_TX, front, address << 8, 16);
    }
    cerr << "\nDone.\n";
    return 0;
}
