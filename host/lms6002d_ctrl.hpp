#ifndef INCLUDED_LMS6002D_CTRL_HPP
#define INCLUDED_LMS6002D_CTRL_HPP

#include <uhd/types/ranges.hpp>
#include <uhd/types/serial.hpp>
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
    static sptr make(uhd::spi_iface::sptr spiface, const int lms_spi_number, const int adf4350_spi_number, const double clock_rate);

    virtual double set_rx_freq(const double freq) = 0;
    virtual double set_tx_freq(const double freq) = 0;

    virtual bool set_rx_enabled(const bool enb) = 0;
    virtual bool set_tx_enabled(const bool enb) = 0;

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
};

#endif /* INCLUDED_LMS6002D_CTRL_HPP */
