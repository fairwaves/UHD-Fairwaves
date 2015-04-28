#ifndef POWER_AMP_HPP
#define POWER_AMP_HPP

#include <uhd/config.hpp>
#include <map>
#include <boost/shared_ptr.hpp>

namespace uhd {

class power_amp {
public:

    // Supported Power Amplifiers
    enum pa_type_t {
        PA_NONE,         // No PA connected
        PA_EPA881F40A,   // EPA881F40A GSM850
        PA_EPA942H40A,   // EPA942H40A EGSM900
        PA_EPA1800F37A   // EPA1800F37A DCS1800
    };

    typedef boost::shared_ptr<power_amp> sptr;
    static power_amp::sptr make(pa_type_t pa_type);
    virtual ~power_amp();

    // Get the PA type
    virtual pa_type_t get_pa_type() const =0;

    // Get the PA type as a string
    virtual std::string get_pa_type_str() const =0;

    // Convert PA type to a string
    static std::string pa_type_to_str(pa_type_t pa);

    // Convert a string to a PA type
    static pa_type_t pa_str_to_type(const std::string &pa_str);

    virtual double min_power() const =0;
    virtual double max_power() const =0;

    virtual double watts_for(double v) const =0;
    virtual double volts_for(double w) const =0;

protected:

    // Map PA types to string names
    struct pa_type_map_pair_t {
        pa_type_t type;
        std::string name;
    };
    static const pa_type_map_pair_t _pa_type_map[];

};

}

#endif // POWER_AMP_HPP
