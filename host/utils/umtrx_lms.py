#!/usr/bin/env python
#
# Copyright 2012 Fairwaves
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import socket, argparse, time
import umtrx_ctrl
# pylint: disable = C0301, C0103, C0111

verbosity = 0

# Constants used during PLL tuning.
VCO_HIGH = 0x02
VCO_NORM = 0x00
VCO_LOW = 0x01

FREQ_LIST = [# min, max, val
    (0.2325e9,   0.285625e9, 0x27),
    (0.285625e9, 0.336875e9, 0x2f),
    (0.336875e9, 0.405e9,    0x37),
    (0.405e9,    0.465e9,    0x3f),
    (0.465e9,    0.57125e9,  0x26),
    (0.57125e9,  0.67375e9,  0x2e),
    (0.67375e9,  0.81e9,     0x36),
    (0.81e9,     0.93e9,     0x3e),
    (0.93e9,     1.1425e9,   0x25),
    (1.1425e9,   1.3475e9,   0x2d),
    (1.3475e9,   1.62e9,     0x35),
    (1.62e9,     1.86e9,     0x3d),
    (1.86e9,     2.285e9,    0x24),
    (2.285e9,    2.695e9,    0x2c),
    (2.695e9,    3.24e9,     0x34),
    (3.24e9,     3.72e9,     0x3c)]

# A list of reserved registers which read as junk
RESV_REGS = (0x0C, 0x0D, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x69, 0x6A, 0x6B, 0x6C, 0x6D)

def dump(lms_dev):
    return [(x, lms_dev.reg_read(x),) for x in range(0, 128) if x not in RESV_REGS]

def select_freq(freq): # test if given freq within the range and return corresponding value
    l = list(filter(lambda t: True if t[0] < freq <= t[1] else False, FREQ_LIST))
    return l[0][2] if len(l) else None

def lms_txrx_pll_tune(lms_dev, base_reg, ref_clock, out_freq):
    """ Tune Tx or RX PLL to a given frequency. Which PLL to tune is selected by
        base_reg parameter: pass 0x10 for TX and 0x20 for RX. """
    freqsel = select_freq(out_freq)
    if freqsel is None:
        print("Error: Output frequency is out of range")
        return False

    vco_x = 1 << ((freqsel & 0x7) - 3)
    nint = int(vco_x * out_freq / ref_clock)
    nfrack = int((1 << 23) * (vco_x * out_freq - nint * ref_clock) / ref_clock)
    print("FREQSEL=%d VCO_X=%d NINT=%d NFRACK=%d" % (freqsel, vco_x, nint, nfrack))

    # Write NINT, NFRAC
    lms_dev.reg_write(base_reg+0x0, (nint >> 1) & 0xff) # NINT[8:1]
    lms_dev.reg_write(base_reg+0x1, ((nfrack >> 16) & 0x7f) | ((nint & 0x1) << 7)) # NINT[0] NFRACK[22:16]
    lms_dev.reg_write(base_reg+0x2, (nfrack >> 8) & 0xff) # NFRACK[15:8]
    lms_dev.reg_write(base_reg+0x3, (nfrack) & 0xff) # NFRACK[7:0]
    # Write FREQSEL
    lms_dev.reg_write(base_reg+0x5, (freqsel << 2) | 0x01) # FREQSEL[5:0] SELOUT[1:0]
    # Reset VOVCOREG, OFFDOWN to default
    lms_dev.reg_write(base_reg+0x8, 0x40) # VOVCOREG[3:1] OFFDOWN[4:0]
    lms_dev.reg_write(base_reg+0x9, 0x94) # VOVCOREG[0] VCOCAP[5:0]

    # Poll VOVCO
    start_i = -1
    stop_i = -1
    state = VCO_HIGH
    for i in range(0, 64):
        lms_dev.reg_write(base_reg+0x9, 0x80 | i)
        comp = lms_dev.reg_read(base_reg+0xa)
        if comp is None:
            return False
        vcocap = comp >> 6
        if verbosity > 1: print("VOVCO[%d]=%x" % (i, vcocap))
        if VCO_HIGH == vcocap:
            pass
        elif VCO_LOW == vcocap:
            if state == VCO_NORM:
                stop_i = i - 1
                state = VCO_LOW
                if verbosity > 1: print("Low")
        elif VCO_NORM == vcocap:
            if state == VCO_HIGH:
                start_i = i
                state = VCO_NORM
                if verbosity > 1: print("Norm")
        else:
            print("ERROR: Incorrect VCOCAP reading while tuning")
            return False
    if VCO_NORM == state:
        stop_i = 63

    if start_i == -1 or stop_i == -1:
        print("ERROR: Can't find VCOCAP value while tuning")
        return False

    # Tune to the middle of the found VCOCAP range
    avg_i = int((start_i + stop_i) / 2)
    print("START=%d STOP=%d SET=%d" % (start_i, stop_i, avg_i))
    lms_dev.reg_write(base_reg+0x9, 0x80 | avg_i)
    return True

def lms_tx_pll_tune(lms_dev, ref_clock, out_freq):
    """ Tune TX PLL to a given frequency. """
    return lms_txrx_pll_tune(lms_dev, 0x10, ref_clock, out_freq)

def lms_rx_pll_tune(lms_dev, ref_clock, out_freq):
    """ Tune TX PLL to a given frequency. """
    return lms_txrx_pll_tune(lms_dev, 0x20, ref_clock, out_freq)

def lms_init(lms_dev):
    """ INIT with default values (taken from the LMS EVB software)"""
    lms_dev.reg_write(0x09, 0x00) # RXOUTSW (disabled), CLK_EN (all disabled)
    lms_dev.reg_write(0x17, 0xE0)
    lms_dev.reg_write(0x27, 0xE3)
    lms_dev.reg_write(0x64, 0x32)
    lms_dev.reg_write(0x70, 0x01)
    lms_dev.reg_write(0x79, 0x37)
    lms_dev.reg_write(0x59, 0x09)
    lms_dev.reg_write(0x47, 0x40)
    # RF Settings
    lms_dev.reg_write(0x41, 0x15) # VGA1GAIN
    lms_dev.reg_write(0x45, 0x00) # VGA2GAIN, ENVD

    # Test settings
#    lms_dev.reg_set_bits(0x35, (1<<6)) # Set BYP_EN_LPF
#    lms_dev.reg_set_bits(0x09, (1<<7)) # Enable RXOUTSW


def lms_tx_enable(lms_dev):
    """ Enable TX """
    # STXEN: Soft transmit enable
#    lms_dev.reg_write(0x05, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1)) # STXEN
    lms_dev.reg_set_bits(0x05, (1 << 3))
    # Tx DSM SPI clock enabled
#    lms_dev.reg_write(0x09, 0x81)
    lms_dev.reg_set_bits(0x09, (1 << 0))

def lms_tx_disable(lms_dev):
    """ Disable TX """
    # STXEN: Soft transmit enable
    lms_dev.reg_clear_bits(0x05, (1 << 3))
    # Tx DSM SPI clock enabled
    lms_dev.reg_clear_bits(0x09, (1 << 0))

def lms_rx_enable(lms_dev):
    """ Enable RX """
    # SRXEN: Soft receive enable
    lms_dev.reg_set_bits(0x05, (1 << 2))
    # Rx DSM SPI clock enabled
    lms_dev.reg_set_bits(0x09, (1 << 2))

def lms_rx_disable(lms_dev):
    """ Disable RX """
    # SRXEN: Soft receive enable
    lms_dev.reg_clear_bits(0x05, (1 << 2))
    # Rx DSM SPI clock enabled
    lms_dev.reg_clear_bits(0x09, (1 << 2))

def lms_get_tx_pa(lms_dev):
    """ Reurn selected Tx PA."""
    return lms_dev.reg_get_bits(0x44, (0x07 << 3), 3)

def lms_set_tx_pa(lms_dev, pa):
    """ Turn on selected Tx PA.
        'pa' parameter is in [0..2] range, where 0 is to turn off all PAs."""
    lms_dev.reg_write_bits(0x44, (0x07 << 3), (pa << 3))

def lms_get_rx_lna(lms_dev):
    """ Reurn selected Rx LNA."""
    #   Note: We should also check register 0x25 here, but it's not clear
    #   what to return if 0x75 and 0x25 registers select different LNAs.

    # LNASEL_RXFE[1:0]: Selects the active LNA.
    return lms_dev.reg_get_bits(0x75, (0x03 << 4), 4)

def lms_set_rx_lna(lms_dev, lna):
    """ Turn on selected Rx LNA.
        'lna' parameter is in [0..3] range, where 0 is to turn off all LNAs."""
    # LNASEL_RXFE[1:0]: Selects the active LNA.
    lms_dev.reg_write_bits(0x75, (0x03 << 4), (lna << 4))
    # SELOUT[1:0]: Select output buffer in RX PLL, not used in TX PLL
    lms_dev.reg_write_bits(0x25, 0x03, lna)

def lms_set_tx_vga1gain(lms_dev, gain):
    """ Set Tx VGA1 gain in dB.
    gain is in [-4 .. -35] dB range
    Returns the old gain value on success, None on error"""
    if not (-35 <= gain <= -4):
        return None
    old_bits = lms_dev.reg_write_bits(0x41, 0x1f, 35 + gain)
    return (old_bits & 0x1f) - 35

def lms_get_tx_vga1gain(lms_dev):
    """ Get Tx VGA1 gain in dB.
    gain is in [-4 .. -35] dB range
    Returns the gain value on success, None on error"""
    return lms_dev.reg_get_bits(0x41, 0x1f, 0)-35

def lms_set_tx_vga2gain(lms_dev, gain):
    """ Set VGA2 gain.
    gain is in dB [0 .. 25]
    Returns the old gain value on success, None on error"""
    if not (0 <= gain <= 25):
        return None
    old_bits = lms_dev.reg_write_bits(0x45, (0x1f << 3), (gain << 3))
    return old_bits >> 3

def lms_get_tx_vga2gain(lms_dev):
    """ Get VGA2 gain in dB.
    gain is in [0 .. 25] dB range
    Returns the gain value on success, None on error"""
    gain = lms_dev.reg_get_bits(0x45, (0x1f << 3), 3)
    gain = gain if gain <= 25 else 25
    return gain

def lms_set_rx_vga2gain(lms_dev, gain):
    """ Set Rx VGA2 gain.
    gain is in dB [0 .. 60]
    Returns the old gain value on success, None on error"""
    if not (0 <= gain <= 60):
        return None
    old_bits = lms_dev.reg_write_bits(0x65, 0x1f, gain/3)
    return (old_bits & 0x1f) * 3

def lms_get_rx_vga2gain(lms_dev):
    """ Get VGA2 gain in dB.
    gain is in [0 .. 60] dB range
    Returns the gain value on success, None on error"""
    gain = lms_dev.reg_get_bits(0x65, 0x1f, 0)
    gain = gain if gain <= 20 else 20
    return gain * 3

def lms_set_vga1dc_i_int(lms_dev, dc_shift_int):
    """ Set VGA1 DC offset, I channel
    dc_shift_int is an integer representation of the DC shift [0 .. 255]
    DC offset = (dc_shift_int - 128) / 16
    Returns the old gain value on success, None on error"""
    if not (0 <= dc_shift_int <= 255):
        return None
    return lms_dev.reg_write_bits(0x42, 0xff, dc_shift_int)

def lms_set_vga1dc_i(lms_dev, dc_shift):
    """ Set VGA1 DC offset, I channel
    dc_shift is an a DC shift in mV [-16 .. 15.9375]
    Returns the old gain value on success, None on error"""
    old_bits = lms_set_vga1dc_i_int(lms_dev, int(dc_shift*16 + 128))
    return (float(old_bits) - 128) / 16

def lms_set_vga1dc_q_int(lms_dev, dc_shift_int):
    """ Set VGA1 DC offset, Q channel
    dc_shift_int is an integer representation of the DC shift [0 .. 255]
    DC offset = (dc_shift_int - 128) / 16
    Returns the old gain value on success, None on error"""
    if not (0 <= dc_shift_int <= 255):
        return None
    return lms_dev.reg_write_bits(0x43, 0xff, dc_shift_int)

def lms_set_vga1dc_q(lms_dev, dc_shift):
    """ Set VGA1 DC offset, Q channel
    dc_shift is an a DC shift in mV [-16 .. 15.9375]
    Returns old gan value on success, None on error"""
    old_bits = lms_set_vga1dc_q_int(lms_dev, int(dc_shift*16 + 128))
    return (float(old_bits) - 128) / 16

# RF Settings for LO leakage tuning
#    lms_dev.reg_write(0x41, (-4 + 35)) # VGA1GAIN
#    lms_dev.reg_write(0x45, (25 << 3) | 0x0) # VGA2GAIN, ENVD
#    lms_dev.reg_write(0x44, (2 << 3) | (1 << 1) | 1) # PA2 on

def lms_general_dc_calibration_loop(lms_dev, dc_addr, calibration_reg_base):
    """ Programming and Calibration Guide: 4.1 General DC Calibration Procedure """
    try_cnt_limit = 10

    if verbosity > 0: print("DC Offset Calibration for addr %d:" % (dc_addr,))
    reg_val = lms_dev.reg_read(calibration_reg_base+0x03)
    # DC_ADDR := ADDR
    reg_val = (reg_val & 0xf8) | dc_addr
    lms_dev.reg_write(calibration_reg_base+0x03, reg_val)
    # DC_START_CLBR := 1
    reg_val = reg_val | (1 << 5)
    lms_dev.reg_write(calibration_reg_base+0x03, reg_val)
    # DC_START_CLBR := 0
    reg_val = reg_val ^ (1 << 5)
    lms_dev.reg_write(calibration_reg_base+0x03, reg_val)

    while try_cnt_limit:
        try_cnt_limit -= 1
        if verbosity > 1: print("cnt=%d" % try_cnt_limit)

        # Wait for 6.4(1.6) us
        time.sleep(6.4e-6)

        # Read DC_CLBR_DONE
        reg_val  = lms_dev.reg_read(calibration_reg_base+0x01)
        DC_CLBR_DONE = (reg_val >> 1) & 0x1
        if verbosity > 1: print(" DC_CLBR_DONE=%d" % DC_CLBR_DONE)

        # DC_CLBR_DONE == 1?
        if DC_CLBR_DONE == 1:
            continue

        # Read DC_LOCK
        reg_val  = lms_dev.reg_read(calibration_reg_base+0x01)
        DC_LOCK = (reg_val >> 2) & 0x7
        if verbosity > 1: print(" DC_LOCK=%d" % DC_LOCK)

        # Read DC_REGVAL
        DC_REGVAL = lms_dev.reg_read(calibration_reg_base+0x00)
        if verbosity > 1: print("DC_REGVAL = %d" % DC_REGVAL)

        # DC_LOCK != 0 or 7?
        if DC_LOCK != 0 and DC_LOCK != 7:
            # We're done.
            break

    # Return the value
    return DC_REGVAL

def lms_general_dc_calibration(lms_dev, dc_addr, calibration_reg_base):
    # This procedure is outlined in FAQ, section 4.7.
    # It's purpose is to circumvent the fact that in some edge cases calibration
    # may be successful even is DC_LOCK shows 0 or 7.

    # Set DC_REGVAL to 31
    lms_dev.reg_write(calibration_reg_base+0x00, 31)
    # Run the calibration first time
    DC_REGVAL = lms_general_dc_calibration_loop(lms_dev, dc_addr, calibration_reg_base)
    # Unchanged DC_REGVAL may mean either calibration failure or that '31' is
    # the best calibration vaue. We're going to re-check that.
    if 31 == DC_REGVAL:
        # Set DC_REGVAL to a value other then 31, e.g. 0
        lms_dev.reg_write(calibration_reg_base+0x00, 0)
        # Retry the calibration
        DC_REGVAL = lms_general_dc_calibration_loop(lms_dev, dc_addr, calibration_reg_base)
        # If DC_REGVAL has been changed, then calibration succeeded.
        if 0 == DC_REGVAL:
            # PANIC: Algorithm does Not Converge!
            # From LimeMicro FAQ: "[This] condition should not happen as this is being
            # checked in our production test."
            print("Error: DC Offset Calibration does not converge!")
            return None

    if verbosity > 0: print("Successful DC Offset Calibration for register bank 0x%X, DC addr %d. Result: 0x%X" \
                            % (calibration_reg_base, dc_addr, DC_REGVAL))
    return DC_REGVAL


def lms_lpf_tuning_dc_calibration(lms_dev):
    """ Programming and Calibration Guide: 4.2 DC Offset Calibration of LPF Tuning Module """
    result = False
    # Save TopSPI::CLK_EN[5] Register
    # TopSPI::CLK_EN[5] := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 5))

    # Perform DC Calibration Procedure in TopSPI with ADDR := 0 and get Result
    # DCCAL := TopSPI::DC_REGVAL
    DCCAL = lms_general_dc_calibration(lms_dev, 0, 0x0)
    if DCCAL is not None:
        # RxLPFSPI::DCO_DACCAL := DCCAL
        lms_dev.reg_write_bits(0x35, 0x3f, DCCAL)
        # TxLPFSPI::DCO_DACCAL := DCCAL
        lms_dev.reg_write_bits(0x55, 0x3f, DCCAL)
        # Success
        result = True

    # Restore TopSPI::CLK_EN[5] Register
    lms_dev.reg_write(0x09, clk_en_save)

    return result

def lms_txrx_lpf_dc_calibration(lms_dev, is_tx):
    """ Programming and Calibration Guide: 4.3 TX/RX LPF DC Offset Calibration """
    # Determine base address for control registers
    control_reg_base = 0x30 if is_tx else 0x50

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 1) if is_tx else (1 << 3))

    # Perform DC Calibration Procedure in LPFSPI with ADDR := 0 (For channel I)
    result = lms_general_dc_calibration(lms_dev, 0, control_reg_base) is not None
    # Perform DC Calibration Procedure in LPFSPI with ADDR := 1 (For channel Q)
    result = lms_general_dc_calibration(lms_dev, 1, control_reg_base) is not None and result

    # Restore TopSPI::CLK_EN Register
    lms_dev.reg_write(0x09, clk_en_save)

    return result

def lms_rxvga2_dc_calibration(lms_dev):
    """ Programming and Calibration Guide: 4.4 RXVGA2 DC Offset Calibration """
    # Set base address for control registers
    control_reg_base = 0x60

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 4))

    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 0 (For DC Reference channel)
    result = lms_general_dc_calibration(lms_dev, 0, control_reg_base) is not None
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 1 (For VGA2A_I channel)
    result = lms_general_dc_calibration(lms_dev, 1, control_reg_base) is not None and result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 2 (For VGA2A_Q channel)
    result = lms_general_dc_calibration(lms_dev, 2, control_reg_base) is not None and result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 3 (For VGA2B_I channel)
    result = lms_general_dc_calibration(lms_dev, 3, control_reg_base) is not None and result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 4 (For VGA2B_Q channel)
    result = lms_general_dc_calibration(lms_dev, 4, control_reg_base) is not None and result

    # Restore TopSPI::CLK_EN Register
    lms_dev.reg_write(0x09, clk_en_save)

    return result

def lms_lpf_bandwidth_tuning(lms_dev, ref_clock, lpf_bandwidth_code):
    """ Programming and Calibration Guide: 4.5 LPF Bandwidth Tuning.
    Note, that this function modifies Tx PLL settings. """
    # Save registers 0x05 and 0x09, because we will modify them during lms_tx_enable()
    reg_save_05 = lms_dev.reg_read(0x05)
    reg_save_09 = lms_dev.reg_read(0x09)

    # Enable TxPLL and tune it to 320MHz
    lms_tx_enable(lms_dev)
    lms_tx_pll_tune(lms_dev, ref_clock, int(320e6))

    # Use 40MHz generatedFrom TxPLL: TopSPI::CLKSEL_LPFCAL := 0
    # Power Up LPF tuning clock generation block: TopSPI::PD_CLKLPFCAL := 0
    reg_save_06 = lms_dev.reg_clear_bits(0x06, (1 << 3) | (1 << 2))

    # Set TopSPI::BWC_LPFCAL
    t = lms_dev.reg_write_bits(0x07, 0x0f, lpf_bandwidth_code)
    if verbosity >= 3: print("code = %x %x %x" % (lpf_bandwidth_code, t, lms_dev.reg_read(0x07)))
    # TopSPI::RST_CAL_LPFCAL := 1 (Rst Active)
    rst_lpfcal_save = lms_dev.reg_set_bits(0x06, 0x01)
    # ...Delay 100ns...
    # TopSPI::RST_CAL_LPFCAL := 0 (Rst Inactive)
    lms_dev.reg_write(0x06, rst_lpfcal_save & ~0x01)
    # RCCAL := TopSPI::RCCAL_LPFCAL
    RCCAL = lms_dev.reg_read(0x01) >> 5
    if verbosity >= 3: print("RCCAL = %d" % RCCAL)
    # RxLPFSPI::RCCAL_LPF := RCCAL
    lms_dev.reg_write_bits(0x56, (7 << 4), (RCCAL << 4))
    # TxLPFSPI::RCCAL_LPF := RCCAL
    lms_dev.reg_write_bits(0x36, (7 << 4), (RCCAL << 4))

    # Restore registers 0x05, 0x06 and 0x09
    lms_dev.reg_write(0x06, reg_save_06)
    lms_dev.reg_write(0x05, reg_save_05)
    lms_dev.reg_write(0x09, reg_save_09)

def lms_auto_calibration(lms_dev, ref_clock, lpf_bandwidth_code):
    """ Performs all automatic calibration procedures in a recommeded order.

        Notes:
          0. Do not forget, that you should not apply any data for Tx during
             the calibration. Rx should be disconnected as well, but we try
             to handle this in the code.
          1. It tunes Tx to 320MHz, so you have to re-tune to your frequency
             after the calibration.
          2. It's better to calibrate with your target TxVGA1 gain. If you
             don't know your target gain yet, choose one <= -7dB to TX mixer
             overload. TxVGA1 gain of -10dB is good choice.
          3. TxVGA2 gain doesn't impact DC offset or LO leakage, because
             it is in RF and is AC coupled. So we don't touch it. Thus TxVGA2
             gain is irrelevant for the purpose of this calibration.
          4. RxVGA2 gain is irrelevant, because it's set to 30dB during the
             calibration and then restored to the original value.
    """
    print("LPF Tuning...")
    lms_lpf_tuning_dc_calibration(lms_dev)
    print("LPF Bandwidth Tuning...")
    lms_lpf_bandwidth_tuning(lms_dev, ref_clock, lpf_bandwidth_code)

    print("Tx LPF DC calibration...")
    lms_txrx_lpf_dc_calibration(lms_dev, True)

    # Disable Rx
    # We use this way of disabling Rx, because we have to leave
    # RXMIX working. If we disable MIX, calibration will not fail,
    # but DC cancellation will be a bit worse. And setting LNASEL_RXFE
    # to 0 disables RXMIX. So instead of that we select LNA1 and then
    # connect it to the internal termination resistor with
    # IN1SEL_MIX_RXFE and RINEN_MIX_RXFE configuration bits.
    lna = lms_get_rx_lna(lms_dev)
    #   1. Select LNA1
    lms_set_rx_lna(lms_dev, 1)
    #   2. Connect LNA to external inputs.
    #      IN1SEL_MIX_RXFE: Selects the input to the mixer
    reg_save_71 = lms_dev.reg_clear_bits(0x71, (1 << 7))
    #   3. Enable internal termination resistor.
    #      RINEN_MIX_RXFE: Termination resistor on external mixer input enable
    reg_save_7C = lms_dev.reg_set_bits(0x7C, (1 << 2))
    # Set RxVGA2 gain to max
    rx_vga2gain = lms_set_rx_vga2gain(lms_dev, 30)
    # Calibrate!
    print("Rx LPF DC calibration...")
    lms_txrx_lpf_dc_calibration(lms_dev, False)
    print("RxVGA2 DC calibration...")
    lms_rxvga2_dc_calibration(lms_dev)

    # Restore saved values
    lms_set_rx_vga2gain(lms_dev, rx_vga2gain)
    lms_dev.reg_write(0x71, reg_save_71)
    lms_dev.reg_write(0x7C, reg_save_7C)
    lms_set_rx_lna(lms_dev, lna)

def enable_loopback(lms_dev):
    """ Enable loopback"""
    lms_dev.reg_set_bits(0x35, (76)) # 
    lms_dev.reg_set_bits(0x64, (28)) # power off RXVGA2
    lms_dev.reg_set_bits(0x09, (192)) # RXOUTSW is closed
    lms_dev.reg_set_bits(0x46, (1 << 2))
    lms_dev.reg_set_bits(0x08, (1 << 4)) # LBEN_OPIN switched on

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'UmTRX LMS debugging tool.', epilog = "UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option. 'None' returned while reading\writing indicates error in the process.")
    parser.add_argument('--version', action='version', version='%(prog)s 3.2')
    parser.add_argument('--lms', type = int, choices = list(range(1, 3)), help = 'LMS number, if no other options are given it will dump all registers for corresponding LMS')
    parser.add_argument('--reg', type = lambda s: int(s, 16), choices = range(0, 0x80), metavar = '0..0x79', help = 'LMS register number, hex')
    parser.add_argument('--verify', action = 'store_true', help = 'read back written register value to verify correctness')
    parser.add_argument('--pll-ref-clock', type = float, default = 26e6, help = 'PLL reference clock, 26MHz by default')
    parser.add_argument('--lpf-bandwidth-code', type = lambda s: int(s, 16), choices = range(0, 0x10), metavar = '0..0x0f', help = 'LPF bandwidth code (default: 0x0f), used only with --lms-lpf-bandwidth-tuning and --lms-auto-calibration')
    basic_opt = parser.add_mutually_exclusive_group()
    basic_opt.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    basic_opt.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')    
    adv_opt = parser.add_mutually_exclusive_group()
    adv_opt.add_argument('--data', type = lambda s: int(s, 16), choices = range(0, 0x100), metavar = '0..0xFF', help = 'data to be written into LMS register, hex')
    adv_opt.add_argument('--dump', action = 'store_true', help = 'dump registers')
    adv_opt.add_argument('--lms-init', action = 'store_true', help = 'run init sequence for LMS')
    adv_opt.add_argument('--lms-tx-enable', type = int, choices = range(0, 2), help = '1 to enable TX chain, 0 to disable TX chain')
    adv_opt.add_argument('--lms-rx-enable', type = int, choices = range(0, 2), help = '1 to enable RX chain, 0 to disable RX chain')
    adv_opt.add_argument('--lms-auto-calibration', action = 'store_true', help = 'A shorthand for --lms-lpf-tuning-dc-calibration, --lms-tx-lpf-dc-calibration, --lms-rx-lpf-dc-calibration, --lms-rxvga2-dc-calibration and --lms-lpf-bandwidth-tuning.')
    adv_opt.add_argument('--lms-lpf-tuning-dc-calibration', action = 'store_true', help = 'DC Offset Calibration of LPF Tuning Module')
    adv_opt.add_argument('--lms-tx-lpf-dc-calibration', action = 'store_true', help = 'TX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rx-lpf-dc-calibration', action = 'store_true', help = 'RX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rxvga2-dc-calibration', action = 'store_true', help = 'RXVGA2 DC Offset Calibration')
    adv_opt.add_argument('--lms-lpf-bandwidth-tuning', action = 'store_true', help = 'LPF bandwidth tuning. Specify --lpf-bandwidth-code to select specific LPF to tune. WARNING: Tx PLL is tuned to 320MHz during this procedure. Don\'t forget to re-tune it back if needed.')
    adv_opt.add_argument('--lms-set-tx-pa',  type = int, choices = range(0, 3), help = 'Activate selected Tx PA, i.e. select LMS output. 0 to turn off all PAs')
    adv_opt.add_argument('--lms-get-tx-pa', action = 'store_true', help = 'Get active PA, i.e. active LMS output. 0 if all outputs are disabled')
    adv_opt.add_argument('--lms-set-rx-lna', type = int, choices = range(0, 4), help = 'Activate selected Rx LNA, i.e. select LMS input. 0 to turn off all LNAs')
    adv_opt.add_argument('--lms-get-rx-lna', action = 'store_true', help = 'Get active LNA, i.e. active LMS input. 0 if all inputs are disabled')
    adv_opt.add_argument('--lms-tx-pll-tune', type = float, metavar = '232.5e6..3720e6', help = 'Tune Tx PLL to the given frequency')
    adv_opt.add_argument('--lms-rx-pll-tune', type = float, metavar = '232.5e6..3720e6', help = 'Tune Rx PLL to the given frequency')
    adv_opt.add_argument('--lms-set-tx-vga1-gain', type = int, choices = range(-35, -3), metavar = '[-35..-4]', help = 'Set Tx VGA1 gain, in dB')
    adv_opt.add_argument('--lms-get-tx-vga1-gain', action = 'store_true', help = 'Get Tx VGA1 gain, in dB')
    adv_opt.add_argument('--lms-set-tx-vga2-gain', type = int, choices = range(0, 26), metavar = '[0..25]', help = 'Set Tx VGA2 gain, in dB')
    adv_opt.add_argument('--lms-get-tx-vga2-gain', action = 'store_true', help = 'Get Tx VGA2 gain, in dB')
    adv_opt.add_argument('--lms-set-rx-vga2-gain', type = int, choices = range(0, 61), metavar = '[0..60]', help = 'Set Rx VGA2 gain, in dB with 3dB accuracy.')
    adv_opt.add_argument('--lms-get-rx-vga2-gain', action = 'store_true', help = 'Get Rx VGA2 gain, in dB')
    adv_opt.add_argument('--lms-tune-vga1-dc-i', action = 'store_true', help = 'Interactive tuning of TxVGA1 DC shift, I channel')
    adv_opt.add_argument('--lms-tune-vga1-dc-q', action = 'store_true', help = 'Interactive tuning of TxVGA1 DC shift, Q channel')
    adv_opt.add_argument('--enable-loopback', action = 'store_true', help = 'enable loopback')
    args = parser.parse_args()
    if args.lms is None: # argparse do not have dependency concept for options
        if args.reg is not None or args.data is not None or args.lms_tx_pll_tune is not None \
           or args.lms_rx_pll_tune is not None or args.lms_init \
           or args.lms_set_tx_pa is not None or args.lms_set_rx_lna is not None \
           or args.lms_get_tx_pa or args.lms_get_rx_lna \
           or args.lms_lpf_tuning_dc_calibration or args.lms_tx_lpf_dc_calibration \
           or args.lms_rx_lpf_dc_calibration or args.lms_rxvga2_dc_calibration \
           or args.lms_auto_calibration or args.lms_lpf_bandwidth_tuning \
           or args.lms_tx_enable is not None or args.lms_tx_enable is not None:
            exit('--lms parameter is required for given options.') # gengetopt is so much better
    if args.data is not None:
        if args.reg is None:
            exit('<data> argument requires <reg> argument.')
    if args.lms_tx_pll_tune is not None:
        if not 232.5e6 < args.lms_tx_pll_tune <= 3720e6:
            exit('<lms-tx-pll-tune> is out of range 232.5e6..3720e6')
    if args.lms_rx_pll_tune is not None:
        if not 232.5e6 < args.lms_rx_pll_tune <= 3720e6:
            exit('<lms-rx-pll-tune> is out of range 232.5e6..3720e6')
    if args.lms_init:
        if args.reg is not None:
            exit('--reg makes no sense with --lms-init, aborting.')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(umtrx_ctrl.UDP_TIMEOUT)
    umtrx = args.umtrx if args.umtrx is not None else umtrx_ctrl.detect(sock, args.bcast_addr)
    if umtrx is not None: # UmTRX address established
        if umtrx_ctrl.ping(sock, umtrx): # UmTRX probed
            umtrx_lms_dev = umtrx_ctrl.umtrx_lms_device(sock, umtrx, args.lms if args.lms else 1)
            if args.lms_init:
                lms_init(umtrx_lms_dev)
            elif args.lms_tx_enable is not None:
                if args.lms_tx_enable == 0:
                    lms_tx_disable(umtrx_lms_dev)
                elif args.lms_tx_enable == 1:
                    lms_tx_enable(umtrx_lms_dev)
                else:
                    print('Wrong parameter value for --lms-tx-enable')
            elif args.lms_rx_enable is not None:
                if args.lms_rx_enable == 0:
                    lms_rx_disable(umtrx_lms_dev)
                elif args.lms_rx_enable == 1:
                    lms_rx_enable(umtrx_lms_dev)
                else:
                    print('Wrong parameter value for --lms-tx-enable')
            elif args.lms_auto_calibration:
                # 0x0f - 0.75MHz
                lpf_bw_code = args.lpf_bandwidth_code if args.lpf_bandwidth_code is not None else 0x0f
                lms_auto_calibration(umtrx_lms_dev, int(args.pll_ref_clock), int(lpf_bw_code))
            elif args.lms_lpf_tuning_dc_calibration:
                lms_lpf_tuning_dc_calibration(umtrx_lms_dev)
            elif args.lms_tx_lpf_dc_calibration:
                lms_txrx_lpf_dc_calibration(umtrx_lms_dev, True)
            elif args.lms_rx_lpf_dc_calibration:
                lms_txrx_lpf_dc_calibration(umtrx_lms_dev, False)
            elif args.lms_rxvga2_dc_calibration:
                lms_rxvga2_dc_calibration(umtrx_lms_dev)
            elif args.lms_set_tx_pa is not None:
                lms_set_tx_pa(umtrx_lms_dev, args.lms_set_tx_pa)
            elif args.lms_get_tx_pa:
                lms_get_tx_pa(umtrx_lms_dev)
            elif args.lms_set_rx_lna is not None:
                lms_set_rx_lna(umtrx_lms_dev, args.lms_set_rx_lna)
            elif args.lms_get_rx_lna:
                lms_get_rx_lna(umtrx_lms_dev)
            elif args.lms_tx_pll_tune is not None:
                lms_tx_pll_tune(umtrx_lms_dev, int(args.pll_ref_clock), int(args.lms_tx_pll_tune))
            elif args.lms_rx_pll_tune is not None:
                lms_rx_pll_tune(umtrx_lms_dev, int(args.pll_ref_clock), int(args.lms_rx_pll_tune))
            elif args.lms_lpf_bandwidth_tuning:
                # 0x0f - 0.75MHz
                lpf_bw_code = args.lpf_bandwidth_code if args.lpf_bandwidth_code is not None else 0x0f
                lms_lpf_bandwidth_tuning(umtrx_lms_dev, int(args.pll_ref_clock), int(lpf_bw_code))
            elif args.lms_set_tx_vga1_gain is not None:
                lms_set_tx_vga1gain(umtrx_lms_dev, int(args.lms_set_tx_vga1_gain))
            elif args.lms_get_tx_vga1_gain:
                gain = lms_get_tx_vga1gain(umtrx_lms_dev)
                print(gain)
            elif args.lms_set_tx_vga2_gain is not None:
                lms_set_tx_vga2gain(umtrx_lms_dev, int(args.lms_set_tx_vga2_gain))
            elif args.lms_get_tx_vga2_gain:
                gain = lms_get_tx_vga2gain(umtrx_lms_dev)
                print(gain)
            elif args.lms_set_rx_vga2_gain is not None:
                lms_set_rx_vga2gain(umtrx_lms_dev, int(args.lms_set_rx_vga2_gain))
            elif args.lms_get_rx_vga2_gain:
                gain = lms_get_rx_vga2gain(umtrx_lms_dev)
                print(gain)
            elif args.lms_tune_vga1_dc_i:
                for i in range(110, 130, 1):
                    print("DC offset %f (%d)" % (float(i-128)/16, i))
                    lms_set_vga1dc_i_int(umtrx_lms_dev, i)
                    time.sleep(1)
            elif args.lms_tune_vga1_dc_q:
                for i in range(103, 150, 1):
                    print("DC offset %f (%d)" % (float(i-128)/16, i))
                    lms_set_vga1dc_q_int(umtrx_lms_dev, i)
                    time.sleep(1)
            elif args.data is not None:
                wrt = umtrx_lms_dev.reg_write(args.reg, args.data)
                if args.verify:
                    vrfy = umtrx_lms_dev.reg_read(args.reg)
                    print('written 0x%02X to REG 0x%02X - %s' % (vrfy, args.reg, 'OK' if vrfy == args.data else 'FAIL'))
            elif args.reg is not None:
                print('read 0x%02X from REG 0x%02X' % (umtrx_lms_dev.reg_read(args.reg), args.reg))
            elif args.enable_loopback:
                enable_loopback(umtrx_lms_dev)
            elif args.lms:
                lms_regs = dump(umtrx_lms_dev)
                print('LMS %u' % args.lms)
                print(''.join('# 0x%02X: 0x%02X\n' % data for data in lms_regs))
            elif args.dump:
                umtrx_lms_dev_1 = umtrx_ctrl.umtrx_lms_device(sock, umtrx, 1)
                umtrx_lms_dev_2 = umtrx_ctrl.umtrx_lms_device(sock, umtrx, 2)
                lms1 = dump(umtrx_lms_dev_1)
                lms2 = dump(umtrx_lms_dev_2)
                diff = list(map(lambda l1, l2: 'OK\n' if l1[1] == l2[1] else 'DIFF\n', lms1, lms2))
                print(''.join(map(lambda l1, l2, d: '# 0x%02X: LMS1=0x%02X \tLMS2=0x%02X\t%s' % (l1[0], l1[1], l2[1], d), lms1, lms2, diff)))
            else:
                print('UmTRX suspected at %s' % umtrx)
        else:
            print('UmTRX at %s is not responding.' % umtrx)
    else:
        print('No UmTRX detected over %s' % args.bcast_addr)
