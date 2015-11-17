// Copyright 2015-2015 Fairwaves LLC
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

#include "umsel2_ctrl.hpp"
#include "umtrx_regs.hpp"

class umsel2_ctrl_impl : public umsel2_ctrl
{
public:

    umsel2_ctrl_impl(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface):
        _ctrl(ctrl), _spiface(spiface)
    {
        //TODO init stuff?
        return;
    }

    ~umsel2_ctrl_impl(void)
    {
        //TODO shutdown
        return;
    }

    uhd::freq_range_t get_rx_freq_range(const int which)
    {
        //TODO range
    }

    double set_rx_freq(const int which, const double freq)
    {
        const int slaveno = (which == 1)?SPI_SS_AUX1 : SPI_SS_AUX2;
        //TODO tune...
        
        return freq;
    }

    uhd::sensor_value_t get_locked(const int which)
    {
        boost::uint32_t irq = _ctrl->peek32(U2_REG_IRQ_RB);
        bool locked = false;
        if (which == 1) locked = (irq & AUX_LD1_IRQ_BIT) != 0;
        if (which == 2) locked = (irq & AUX_LD2_IRQ_BIT) != 0;
        return uhd::sensor_value_t("LO", locked, "locked", "unlocked");
    }

private:
    uhd::wb_iface::sptr _ctrl;
    uhd::spi_iface::sptr _spiface;
};

umsel2_ctrl::sptr umsel2_ctrl::make(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface)
{
    return umsel2_ctrl::sptr(new umsel2_ctrl_impl(ctrl, spiface));
}
