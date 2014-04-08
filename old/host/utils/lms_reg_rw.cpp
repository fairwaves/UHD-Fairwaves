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
#include <stdint.h>
#include <uhd/utils/safe_main.hpp>
#include <uhd/device.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/property_tree.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <cstdio>
#include <iostream>
#include "../lib/usrp/umtrx/umtrx_regs.hpp"

namespace po = boost::program_options;
namespace uu = uhd::usrp;
using namespace std;

const string version = "0.0.1";

uint32_t reg_read(uu::dboard_iface::sptr dbif, uu::dboard_iface::unit_t lms, uhd::spi_config_t front, uint8_t addr) {
    return dbif->read_write_spi(lms, front, addr << 8, 16);
}

uint32_t reg_write(uu::dboard_iface::sptr dbif, uu::dboard_iface::unit_t lms, uhd::spi_config_t front, uint8_t addr, uint8_t data) {
    return dbif->read_write_spi(lms, front, (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data, 16);
}

uu::dboard_iface::sptr get_dbif(string args) {// TX interface, card 0 hardcoded - in hardware this should be equal to RX
    uu::multi_usrp::sptr usrp = uu::multi_usrp::make(args);
    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    const string mb_name = tree->access<string>("/mboards/0/name").get();
    if (mb_name.find("UMTRX") == string::npos and mb_name.find("UmTRX") == string::npos) {
	cerr << "No supported hardware found.\n";
	exit(1);
    }
    return usrp->get_tx_dboard_iface(0);
}

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
	("verbose,v", "print additional information besides actual register value")
	("version", "print version information")
	("dump", "dump all registers from both LMS and compare results")
        ("fall", "use FALL signal edge for SPI, defaults to RISE");
// N. B: using po::value<uint8_t> causes boost to crap and ignore correct option value
// I miss GNU/gengetopt so much...
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("version")) cerr << boost::format("LMS register dumper version %s\n") % version;
    if (vm.count("help") or not (vm.count("address") or vm.count("dump"))) {
        cerr << boost::format("Options: %s\n") % desc ;
        cerr << "Omit the data argument to perform a readback,\nOr specify a new data to write into the address.\n";
        return 2;
    }

    uhd::spi_config_t front = vm.count("fall")?(uhd::spi_config_t::EDGE_FALL):(uhd::spi_config_t::EDGE_RISE);
    if (vm.count("verbose")) cerr << "Creating UmTRX device from address: " << args << "\n";
    uu::dboard_iface::sptr dbif = get_dbif(args);

    if (vm.count("dump")) {
	for (int i = 0; i < 128; i++) {
	    uint32_t lms1 = reg_read(dbif, (uu::dboard_iface::unit_t)1, front, i);
	    uint32_t lms2 = reg_read(dbif, (uu::dboard_iface::unit_t)2, front, i);
	    printf("# %.3u: LMS1=%X\tLMS2=%X\t%s\n", i, lms1, lms2, (lms1 == lms2)?(" OK"):(" DIFF"));
	}
	return 0;
    }

    if (vm["address"].as<unsigned>() > 127) {
	cerr << "Expected register address is [0; 127], received " << vm["address"].as<unsigned>() << "\n";
	return 3;
    }
    
    uint8_t address = vm["address"].as<unsigned>();

    if ((1 != lms) && (2 != lms)) {
	cerr << "Unexpected LMS number supplied: " << lms << ", only 1 & 2 are available.\n";
	return 4;
    } 

    if(vm.count("verbose")) fprintf(stderr, "Using %s SPI on LMS unit %u\n", vm.count("fall")?("EDGE_FALL"):("EDGE_RISE"), lms);

    if (vm.count("data")) {
	if (vm["data"].as<unsigned>() > 255) {
	    cerr << "Expected data for register is [0; 127], received " << vm["data"].as<unsigned>() << "\n";
	    return 5;
	}
	uint8_t data = vm["data"].as<unsigned>();
        if(vm.count("verbose")) cerr << "Writing " << hex << data << " to register " << hex << address << "... ";
	if(vm.count("verbose")) cerr << reg_write(dbif, (uu::dboard_iface::unit_t)lms, front, address, data);
	else cout << reg_write(dbif, (uu::dboard_iface::unit_t)lms, front, address, data);
    } else {
	if(vm.count("verbose")) cerr << "Reading register " << hex << address << "... ";
	if(vm.count("verbose")) cerr << reg_read(dbif, (uu::dboard_iface::unit_t)lms, front, address);
	else cout << reg_read(dbif, (uu::dboard_iface::unit_t)lms, front, address);
    }
    if(vm.count("verbose")) cerr << "\nDone.";
    cerr << endl;
    return 0;
}
