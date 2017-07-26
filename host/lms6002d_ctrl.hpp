#ifndef INCLUDED_LMS6002D_CTRL_HPP
#define INCLUDED_LMS6002D_CTRL_HPP

#include <uhd/types/ranges.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

/***********************************************************************
 * The interface to lms6002d that get registered into the property tree
 **********************************************************************/
class lms6002d_ctrl
{
public:
    typedef boost::shared_ptr<lms6002d_ctrl> sptr;
    static sptr make(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate);

    virtual double set_rx_freq(const double freq) = 0;
    virtual double set_tx_freq(const double freq) = 0;

    virtual bool set_rx_enabled(const bool enb) = 0;
    virtual bool set_tx_enabled(const bool enb) = 0;

    virtual uhd::sensor_value_t get_rx_pll_locked() = 0;
    virtual uhd::sensor_value_t get_tx_pll_locked() = 0;

    virtual double set_rx_gain(const double gain, const std::string &name) = 0;
    virtual double set_tx_gain(const double gain, const std::string &name) = 0;

    virtual void set_rx_ant(const std::string &ant) = 0;
    virtual void set_tx_ant(const std::string &ant) = 0;

    virtual double set_rx_bandwidth(const double bandwidth) = 0;
    virtual double set_tx_bandwidth(const double bandwidth) = 0;

    virtual uhd::freq_range_t get_rx_bw_range(void) = 0;
    virtual uhd::freq_range_t get_tx_bw_range(void) = 0;

    virtual uhd::freq_range_t get_rx_freq_range(void) = 0;
    virtual uhd::freq_range_t get_tx_freq_range(void) = 0;

    virtual std::vector<std::string> get_tx_antennas(void) = 0;
    virtual std::vector<std::string> get_rx_antennas(void) = 0;

    virtual std::vector<std::string> get_tx_gains(void) = 0;
    virtual std::vector<std::string> get_rx_gains(void) = 0;

    virtual uhd::gain_range_t get_rx_gain_range(const std::string &name) = 0;
    virtual uhd::gain_range_t get_tx_gain_range(const std::string &name) = 0;

    virtual uint8_t _set_tx_vga1dc_i_int(uint8_t offset) = 0;
    virtual uint8_t _set_tx_vga1dc_q_int(uint8_t offset) = 0;

    virtual uint8_t get_tx_vga1dc_i_int(void) = 0;
    virtual uint8_t get_tx_vga1dc_q_int(void) = 0;

    virtual void set_rxfe_dc_i(uint8_t value) = 0;
    virtual uint8_t get_rxfe_dc_i() = 0;
    virtual void set_rxfe_dc_q(uint8_t value) = 0;
    virtual uint8_t get_rxfe_dc_q() = 0;
    virtual void set_rxlpf_dc_i(uint8_t value) = 0;
    virtual uint8_t get_rxlpf_dc_i() = 0;
    virtual void set_rxlpf_dc_q(uint8_t value) = 0;
    virtual uint8_t get_rxlpf_dc_q() = 0;
    virtual void set_rxvga2_dc_reference(uint8_t value) = 0;
    virtual uint8_t get_rxvga2_dc_reference() = 0;
    virtual void set_rxvga2a_dc_i(uint8_t value) = 0;
    virtual uint8_t get_rxvga2a_dc_i() = 0;
    virtual void set_rxvga2a_dc_q(uint8_t value) = 0;
    virtual uint8_t get_rxvga2a_dc_q() = 0;
    virtual void set_rxvga2b_dc_i(uint8_t value) = 0;
    virtual uint8_t get_rxvga2b_dc_i() = 0;
    virtual void set_rxvga2b_dc_q(uint8_t value) = 0;
    virtual uint8_t get_rxvga2b_dc_q() = 0;
};

#endif /* INCLUDED_LMS6002D_CTRL_HPP */
