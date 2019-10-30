#include "power_amp.hpp"
#include "umtrx_log_adapter.hpp"
#include <uhd/exception.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <cmath>

using namespace uhd;

/***********************************************************************
 * Declarations
 **********************************************************************/

class power_amp_impl : public power_amp {
public:

    // Voltage to Watts curves
    typedef std::map<double,double> pa_curve_t;

    power_amp_impl(pa_type_t pa_type, const pa_curve_t &v2w, const pa_curve_t &w2v);
    virtual ~power_amp_impl();

    // Get the PA type
    virtual pa_type_t get_pa_type() const {return _pa_type;}

    // Get the PA type as a string
    virtual std::string get_pa_type_str() const {return pa_type_to_str(_pa_type);}

    // Minimum and maximum supported output power in watts
    virtual double min_power_w() const;
    virtual double max_power_w() const;
    // Minimum and maximum supported output power in dBm
    virtual double min_power_dBm() const;
    virtual double max_power_dBm() const;

    // Get output power in watts for a given voltage
    virtual double v2w(double v) const;
    // Get input voltage required to generate given output power (in watts)
    virtual double w2v(double w) const;
    // Get output power in dBm for a given voltage
    virtual double v2dBm(double v) const;
    // Get input voltage required to generate given output power (in dBm)
    virtual double dBm2v(double dBm) const;

protected:

    // The PA type
    pa_type_t _pa_type;

    // Curves
    const pa_curve_t &_v2w_curve;
    const pa_curve_t _w2v_curve;
};

// Interpolate curve values
static double pa_interpolate_curve(const power_amp_impl::pa_curve_t &curve, double v);
// Convert an A->B map into a B->A map
template<typename map_t> static map_t map_reverse(map_t curve);

/***********************************************************************
 * Constants
 **********************************************************************/

// NOTE: All names MUST be uppercase for pa_str_to_type() to work correctly
const power_amp::pa_type_map_pair_t power_amp::_pa_type_map[] = {
    {power_amp::PA_NONE, "NONE"}, // Also serves as the default
    {power_amp::PA_EPA881F40A, "EPA881F40A"},
    {power_amp::PA_EPA942H40A, "EPA942H40A"},
    {power_amp::PA_EPA1800F37A, "EPA1800F37A"}
};

const power_amp_impl::pa_curve_t EPA942H40A_v2w_curve = boost::assign::map_list_of
    (9.5,   1.15)
    (10,    1.31)
    (11,    1.6)
    (12,    1.9)
    (12.5,  2.1)
    (13,    2.25)
    (13.5,  2.44)
    (14,    2.6)
    (14.5,  2.8)
    (15,    3.0)
    (15.5,  3.2)
    (16,    3.45)
    (16.5,  3.7)
    (17,    3.9)
    (17.5,  4.1)
    (18,    4.35)
    (18.5,  4.6)
    (19,    4.8)
    (19.5,  5.1)
    (20,    5.4)
    (20.5,  5.65)
    (21.1,  6.0)
    (21.6,  6.2)
    (22.1,  6.5)
    (22.6,  6.8)
    (23.1,  7.1)
    (23.4,  7.25)
    (23.7,  7.4)
    (24,    7.55)
    (24.2,  7.7)
    (24.5,  7.9)
    (24.8,  8.0)
    (25.2,  8.25)
    (25.5,  8.45)
    (25.9,  8.65)
    (26.2,  8.9)
    (26.6,  9.1)
    (28,   10.0);

/***********************************************************************
 * Static functions
 **********************************************************************/

template<typename map_t>
static map_t map_reverse(map_t curve)
{
    map_t reversed;

    for (typename map_t::iterator i = curve.begin(); i != curve.end(); ++i)
        reversed[i->second] = i->first;

    return reversed;
}

static double pa_interpolate_curve(const power_amp_impl::pa_curve_t &curve, double v)
{
    power_amp_impl::pa_curve_t::const_iterator i = curve.upper_bound(v);
    if (i == curve.end())
    {
        return (--i)->second;
    }
    if (i == curve.begin())
    {
        return i->second;
    }
    power_amp_impl::pa_curve_t::const_iterator l=i;
    --l;

    const double delta=(v - l->first) / (i->first - l->first);
    return delta*i->second + (1-delta)*l->second;
}

/***********************************************************************
 * Make
 **********************************************************************/
power_amp::sptr power_amp::make(pa_type_t pa_type) {
    switch (pa_type) {
    case PA_NONE:
        return power_amp::sptr();
    case PA_EPA881F40A:
    case PA_EPA942H40A:
    case PA_EPA1800F37A:
    default:
        return power_amp::sptr(new power_amp_impl(pa_type, EPA942H40A_v2w_curve,
                                                  map_reverse(EPA942H40A_v2w_curve)));
    }
}

/***********************************************************************
 * power_amp methods
 **********************************************************************/

power_amp::~power_amp()
{
}

std::string power_amp::pa_type_to_str(pa_type_t pa)
{
    BOOST_FOREACH(const power_amp::pa_type_map_pair_t &pa_map_pair, _pa_type_map)
    {
        if (pa_map_pair.type == pa)
            return pa_map_pair.name;
    }
    throw uhd::environment_error("Can't map PA type to a string.");
    return "NONE";
}

power_amp::pa_type_t power_amp::pa_str_to_type(const std::string &pa_str)
{
    std::string pa_str_upper = boost::to_upper_copy(pa_str);
    BOOST_FOREACH(const power_amp::pa_type_map_pair_t &pa_map_pair, _pa_type_map)
    {
        if (pa_map_pair.name == pa_str_upper)
            return pa_map_pair.type;
    }
    UHD_MSG(error) << "PA name " << pa_str << " is not recognized. "
                   << "Setting PA type to NONE." << std::endl;
    return power_amp::PA_NONE;
}

std::list<power_amp::pa_type_t> power_amp::list_pa_type()
{
    std::list<power_amp::pa_type_t> list;

    BOOST_FOREACH(const power_amp::pa_type_map_pair_t &pa_map_pair, _pa_type_map) {
        list.push_back(pa_map_pair.type);
    }

    return list;
}

std::list<std::string> power_amp::list_pa_str()
{
    std::list<std::string> list;

    BOOST_FOREACH(const power_amp::pa_type_map_pair_t &pa_map_pair, _pa_type_map) {
        list.push_back(pa_map_pair.name);
    }

    return list;
}

double power_amp::w2dBm(double w)
{
    return 10*log10(w) + 30;
}

double power_amp::dBm2w(double dBm)
{
    return pow(10, (dBm-30)/10);
}

/***********************************************************************
 * power_amp_impl methods
 **********************************************************************/

power_amp_impl::power_amp_impl(pa_type_t pa_type, const pa_curve_t &v2w, const pa_curve_t &w2v)
    : _pa_type(pa_type)
    , _v2w_curve(v2w)
    , _w2v_curve(w2v)
{

}

power_amp_impl::~power_amp_impl()
{
}

double power_amp_impl::min_power_w() const
{
    return _w2v_curve.begin()->first;
}

double power_amp_impl::max_power_w() const
{
    return _w2v_curve.rbegin()->first;
}

double power_amp_impl::min_power_dBm() const
{
    return w2dBm(min_power_w());
}

double power_amp_impl::max_power_dBm() const
{
    return w2dBm(max_power_w());
}

double power_amp_impl::v2w(double v) const
{
    return pa_interpolate_curve(_v2w_curve, v);
}

double power_amp_impl::w2v(double w) const
{
    return pa_interpolate_curve(_w2v_curve, w);
}

double power_amp_impl::v2dBm(double v) const
{
    return w2dBm(v2w(v));
}

double power_amp_impl::dBm2v(double dBm) const
{
    return w2v(dBm2w(dBm));
}
