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

#ifndef INCLUDED_UMTRX_IFACE_HPP
#define INCLUDED_UMTRX_IFACE_HPP

#include <uhd/transport/udp_simple.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/types/wb_iface.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <string>

#include "umtrx_common.hpp"

/*!
 * The umtrx interface class:
 * Provides a set of functions to implementation layer.
 * Including spi, peek, poke, control...
 */
class umtrx_iface : public uhd::wb_iface, public uhd::spi_iface, public uhd::i2c_iface{
public:
    typedef UMTRX_UHD_PTR_NAMESPACE::shared_ptr<umtrx_iface> sptr;
    /*!
     * Make a new umtrx interface with the control transport.
     * \param ctrl_transport the udp transport object
     * \return a new umtrx interface object
     */
    static sptr make(uhd::transport::udp_simple::sptr ctrl_transport);

    //! The list of possible revision types
    enum rev_type {
        UMTRX_REV0 = 64000,
        USRP_NXXX = 0
    };

    //! Get the revision type for this device
    virtual rev_type get_rev(void) = 0;

    //! Get the canonical name for this device
    virtual const std::string get_cname(void) = 0;

    //! Lock the device to this iface
    virtual void lock_device(bool lock) = 0;

    //! Is this device locked?
    virtual bool is_device_locked(void) = 0;

    //! A version string for firmware
    virtual const std::string get_fw_version_string(void) = 0;

    //! A hack: Perform an action on the ZPU
    virtual uint32_t send_zpu_action(uint32_t action, uint32_t data) = 0;

    //motherboard eeprom map structure
    uhd::usrp::mboard_eeprom_t mb_eeprom;
};

#endif /* INCLUDED_UMTRX_IFACE_HPP */
