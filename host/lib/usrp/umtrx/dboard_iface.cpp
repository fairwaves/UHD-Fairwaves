//
// Copyright 2012 Fairwaves
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
#include "gpio_core_200.hpp"
#include "umtrx_regs.hpp" //wishbone address constants?
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/algorithm.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/asio.hpp> //htonl and ntohl
#include <boost/math/special_functions/round.hpp>
#include "../usrp2/usrp2_iface.hpp"
#include <cstdio>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

class umtrx_dboard_iface : public dboard_iface {
    usrp2_iface::sptr _iface;
    const std::string _dboard;
    double _ref_clk;
    int _lms_spi_number;
    int _adf4350_spi_number;
    void _write_aux_dac(unit_t) {}

public:
    umtrx_dboard_iface(usrp2_iface::sptr iface, const std::string board, double ref_clk)
        : _iface(iface), _dboard(board),
          _ref_clk(ref_clk),
          _lms_spi_number(board=="A"?SPI_SS_LMS1:SPI_SS_LMS2),
          _adf4350_spi_number(board=="A"?SPI_SS_AUX1:SPI_SS_AUX2)
    {}
    ~umtrx_dboard_iface(void) {}

    special_props_t get_special_props(void) {
        special_props_t props;
        props.soft_clock_divider = false;
        props.mangle_i2c_addrs = false;
        return props;
    }

    void write_aux_dac(unit_t, aux_dac_t, double) {}
    double read_aux_adc(unit_t, aux_adc_t) { return 0; }

    void _set_pin_ctrl(unit_t, boost::uint16_t) {}
    void _set_gpio_ddr(unit_t, boost::uint16_t) {}
    void _set_gpio_out(unit_t, boost::uint16_t) {}
    boost::uint16_t read_gpio(unit_t) { return 0; }
    void _set_atr_reg(unit_t, atr_reg_t, boost::uint16_t) {}
    void set_gpio_debug(unit_t, int) {}

    void write_i2c(boost::uint8_t addr, const byte_vector_t &bytes) { return _iface->write_i2c(addr, bytes); }
    byte_vector_t read_i2c(boost::uint8_t addr, size_t num_bytes) { return _iface->read_i2c(addr, num_bytes); }

    void set_clock_rate(unit_t, double) { /* The clock rate is fixed */ }
    double get_clock_rate(unit_t) { return _ref_clk; }
    std::vector<double> get_clock_rates(unit_t) { assert(!"umtrx_dboard_iface::get_clock_rates() is not implemented"); std::vector<double> FIXME; return FIXME; }
    void set_clock_enabled(unit_t, bool) { /* It's always enabled. */ }
    double get_codec_rate(unit_t) { assert(!"umtrx_dboard_iface::get_codec_rate() is not implemented"); return 0; }

    void write_spi(unit_t spi_device, const spi_config_t &config, boost::uint32_t data, size_t num_bits) {
        if (spi_device == uhd::usrp::dboard_iface::UNIT_LMS) {
            // Access LMS
            _iface->write_spi(_lms_spi_number, config, data, num_bits);
        } else if (spi_device == uhd::usrp::dboard_iface::UNIT_SYNT) {
            // Access ADF4350 synthetiser
//            _iface->write_spi(_adf4350_spi_number, config, data, num_bits);
            _iface->write_spi(SPI_SS_AUX1, config, data, num_bits);
            _iface->write_spi(SPI_SS_AUX2, config, data, num_bits);
        } else {
            assert(!"Wrong unit_t pased to umtrx_dboard_iface::write_spi.");
        }
    }

    boost::uint32_t read_write_spi(unit_t spi_device, const spi_config_t &config, boost::uint32_t data, size_t num_bits) {
        if (spi_device == uhd::usrp::dboard_iface::UNIT_LMS) {
            // Access LMS
            return _iface->read_spi(_lms_spi_number, config, data, num_bits);
        } else if (spi_device == uhd::usrp::dboard_iface::UNIT_SYNT) {
            // Access ADF4350 synthetiser
//            return _iface->read_spi(_adf4350_spi_number, config, data, num_bits);
            return _iface->read_spi(SPI_SS_AUX1, config, data, num_bits);
            return _iface->read_spi(SPI_SS_AUX2, config, data, num_bits);
        } else {
            assert(!"Wrong unit_t pased to umtrx_dboard_iface::write_spi.");
        }
    }
};

/**********************************
 * Make Function
 **********************************/
dboard_iface::sptr make_umtrx_dboard_iface(usrp2_iface::sptr iface, const std::string dboard, double ref_clk) {
    return dboard_iface::sptr(new umtrx_dboard_iface(iface, dboard, ref_clk));
}
