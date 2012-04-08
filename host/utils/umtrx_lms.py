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

def dump(lms_dev):
    return [lms_dev.reg_read(x) for x in range(0, 128)]

def select_freq(freq): # test if given freq within the range and return corresponding value
    l = list(filter(lambda t: True if t[0] < freq <= t[1] else False, FREQ_LIST))
    return l[0][2] if len(l) else None

def lms_pll_tune(lms_dev, ref_clock, out_freq):
    freqsel = select_freq(out_freq)
    if freqsel is None:
        print("Error: Output frequency is out of range")
        return False

    vco_x = 1 << ((freqsel & 0x7) - 3)
    nint = int(vco_x * out_freq / ref_clock)
    nfrack = int((1 << 23) * (vco_x * out_freq - nint * ref_clock) / ref_clock)
    print("FREQSEL=%d VCO_X=%d NINT=%d NFRACK=%d" % (freqsel, vco_x, nint, nfrack))

    # Write NINT, NFRAC
    lms_dev.reg_write(0x10, (nint >> 1) & 0xff) # NINT[8:1]
    lms_dev.reg_write(0x11, ((nfrack >> 16) & 0x7f) | ((nint & 0x1) << 7)) # NINT[0] NFRACK[22:16]
    lms_dev.reg_write(0x12, (nfrack >> 8) & 0xff) # NFRACK[15:8]
    lms_dev.reg_write(0x13, (nfrack) & 0xff) # NFRACK[7:0]
    # Write FREQSEL
    lms_dev.reg_write(0x15, (freqsel << 2) | 0x01) # FREQSEL[5:0] SELOUT[1:0]
    # Reset VOVCOREG, OFFDOWN to default
    lms_dev.reg_write(0x18, 0x40) # VOVCOREG[3:1] OFFDOWN[4:0]
    lms_dev.reg_write(0x19, 0x94) # VOVCOREG[0] VCOCAP[5:0]

    # Poll VOVCO
    start_i = -1
    stop_i = -1
    state = VCO_HIGH
    for i in range(0, 64):
        lms_dev.reg_write(0x19, 0x80 | i)
        comp = lms_dev.reg_read(0x1a)
        if comp is None:
            return False
        vcocap = comp >> 6
        print("VOVCO[%d]=%x" % (i, vcocap))
        if VCO_HIGH == vcocap:
            pass
        elif VCO_LOW == vcocap:
            if state == VCO_NORM:
                stop_i = i - 1
                state = VCO_LOW
                print("Low")
        elif VCO_NORM == vcocap:
            if state == VCO_HIGH:
                start_i = i
                state = VCO_NORM
                print("Norm")
        else:
            print("ERROR WHILE TUNING")
            return False

    if start_i == -1 or stop_i == -1:
        print("CAN'T TUNE")
        return False
    # Tune to the middle of the found VCOCAP range
    avg_i = int((start_i + stop_i) / 2)
    print("START=%d STOP=%d SET=%d" % (start_i, stop_i, avg_i))
    lms_dev.reg_write(0x19, 0x80 | avg_i)
    return True

def lms_init(lms_dev):
    """ INIT with default values (taken from the LMS EVB software)"""
#    lms_dev.reg_write(0x09, 0xC0) # questionable step
    lms_dev.reg_write(0x09, 0x80)
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

def lms_tx_enable(lms_dev):
    """Enable TX"""
    # Enable STXEN: Soft transmit enable
#    lms_dev.reg_write(0x05, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1)) # STXEN
    lms_dev.reg_set_bits(0x05, (1 << 3))
    # Set Tx DSM SPI clock enabled
#    lms_dev.reg_write(0x09, 0x81)
    lms_dev.reg_set_bits(0x09, (1 << 0))

def lms_pa_off(lms_dev):
    lms_dev.reg_clear_bits(0x44, (0x07 << 3))

def lms_pa_on(lms_dev, pa):
    """ Turn on PA, 'pa' parameter is in [1..2] range"""
    lms_dev.reg_write_bits(0x44, (0x07 << 3), (pa << 3))

# RF Settings for LO leakage tuning
#    lms_dev.reg_write(0x41, (-4 + 35)) # VGA1GAIN
#    lms_dev.reg_write(0x45, (25 << 3) | 0x0) # VGA2GAIN, ENVD
#    lms_dev.reg_write(0x44, (2 << 3) | (1 << 1) | 1) # PA2 on

def lms_general_dc_calibration(lms_dev, dc_addr, calibration_reg_base):
    """ Programming and Calibration Guide: 4.1 General DC Calibration Procedure """
    try_cnt_limit = 10

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
#        print("cnt=%d" % try_cnt_limit)

        # Wait for 6.4(1.6) us
        time.sleep(6.4e-6)

        # Read DC_CLBR_DONE
        reg_val  = lms_dev.reg_read(calibration_reg_base+0x01)
        DC_CLBR_DONE = (reg_val >> 1) & 0x1
#        print(" DC_CLBR_DONE=%d" % DC_CLBR_DONE)

        # DC_CLBR_DONE == 1?
        if DC_CLBR_DONE:
            continue

        # Read DC_LOCK
        reg_val  = lms_dev.reg_read(calibration_reg_base+0x01)
        DC_LOCK = (reg_val >> 2) & 0x7
#        print(" DC_LOCK=%d" % DC_LOCK)

        # DC_LOCK != 0 or 7?
        if DC_LOCK != 0 and DC_LOCK != 7:
            # Read DC_REGVAL
            DC_REGVAL = lms_dev.reg_read(calibration_reg_base+0x00)
            print("DC_REGVAL = %d" % DC_REGVAL)
            return DC_REGVAL

    # PANIC: Algorithm does Not Converge!
    print("Error: DC Offset Calibration does not converge!")
    return None

def lms_lpf_tuning_dc_calibration(lms_dev):
    """ Programming and Calibration Guide: 4.2 DC Offset Calibration of LPF Tuning Module """
    # Save TopSPI::CLK_EN[5] Register
    # TopSPI::CLK_EN[5] := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 5))

    # Perform DC Calibration Procedure in TopSPI with ADDR := 0 and get Result
    # DCCAL := TopSPI::DC_REGVAL
    DCCAL = lms_general_dc_calibration(lms_dev, 0, 0x0)
    if DCCAL is None:
        # Restore TopSPI::CLK_EN[5] Register
        lms_dev.reg_write(0x09, clk_en_save)
        return False

    # RxLPFSPI::DCO_DACCAL := DCCAL
    lms_dev.reg_write_bits(0x35, 0x3f, DCCAL)

    # TxLPFSPI::DCO_DACCAL := DCCAL
    lms_dev.reg_write_bits(0x55, 0x3f, DCCAL)

    # Restore TopSPI::CLK_EN[5] Register
    lms_dev.reg_write(0x09, clk_en_save)
    return True

def lms_txrx_lpf_dc_calibration(lms_dev, is_tx):
    """ Programming and Calibration Guide: 4.3 TX/RX LPF DC Offset Calibration """
    # Determine base address for control registers
    control_reg_base = 0x30 if is_tx else 0x50

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 1) if is_tx else (1 << 3))

    # Perform DC Calibration Procedure in LPFSPI with ADDR := 0 (For channel I) and get Result
    # Perform DC Calibration Procedure in LPFSPI with ADDR := 1 (For channel Q) and get Result
    if lms_general_dc_calibration(lms_dev, 0, control_reg_base) is None \
       or lms_general_dc_calibration(lms_dev, 1, control_reg_base) is None:
        # Restore TopSPI::CLK_EN Register
        lms_dev.reg_write(0x09, clk_en_save)
        return False

    # Restore TopSPI::CLK_EN Register
    lms_dev.reg_write(0x09, clk_en_save)
    return True

def lms_rxvga2_dc_calibration(lms_dev):
    """ Programming and Calibration Guide: 4.4 RXVGA2 DC Offset Calibration """
    # Set base address for control registers
    control_reg_base = 0x60

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = lms_dev.reg_set_bits(0x09, (1 << 4))

    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 0 (For DC Reference channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 1 (For VGA2A_I channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 2 (For VGA2A_Q channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 3 (For VGA2B_I channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 4 (For VGA2B_Q channel) and get Result
    if lms_general_dc_calibration(lms_dev, 0, control_reg_base) is None \
       or lms_general_dc_calibration(lms_dev, 1, control_reg_base) is None \
       or lms_general_dc_calibration(lms_dev, 2, control_reg_base) is None \
       or lms_general_dc_calibration(lms_dev, 3, control_reg_base) is None \
       or lms_general_dc_calibration(lms_dev, 4, control_reg_base) is None:
        # Restore TopSPI::CLK_EN Register
        lms_dev.reg_write(0x09, clk_en_save)
        return False

    # Restore TopSPI::CLK_EN Register
    lms_dev.reg_write(0x09, clk_en_save)
    return True

def lms_lpf_bandwidth_tuning(lms_dev, ref_clock, lpf_bandwidth_code):
    """ Programming and Calibration Guide: 4.5 LPF Bandwidth Tuning.
    Note, that this function modifies Tx PLL settings. """
    # Save registers 0x05 and 0x09, because we will modify them during lms_tx_enable()
    reg_save_05 = lms_dev.reg_read(0x05)
    reg_save_09 = lms_dev.reg_read(0x09)

    # Enable TxPLL and set toProduce 320MHz
    lms_tx_enable(lms_dev)
    lms_pll_tune(lms_dev, ref_clock, int(320e6))

    # Use 40MHz generatedFrom TxPLL: TopSPI::CLKSEL_LPFCAL := 0
    # Power Up LPF tuning clock generation block: TopSPI::PD_CLKLPFCAL := 0
    reg_save_06 = lms_dev.reg_clear_bits(0x06, (1 << 3) | (1 << 2))

    # Set TopSPI::BWC_LPFCAL
    t = lms_dev.reg_write_bits(0x07, 0x0f, lpf_bandwidth_code)
    print("code = %x %x %x" % (lpf_bandwidth_code, t, lms_dev.reg_read(0x07)))
    # TopSPI::RST_CAL_LPFCAL := 1 (Rst Active)
    rst_lpfcal_save = lms_dev.reg_set_bits(0x06, 0x01)
    # ...Delay 100ns...
    # TopSPI::RST_CAL_LPFCAL := 0 (Rst Inactive)
    lms_dev.reg_write(0x06, rst_lpfcal_save & ~0x01)
    # RCCAL := TopSPI::RCCAL_LPFCAL
    RCCAL = lms_dev.reg_read(0x01) >> 5
    print("RCCAL = %d" % RCCAL)
    # RxLPFSPI::RCCAL_LPF := RCCAL
    lms_dev.reg_write_bits(0x56, (7 << 4), (RCCAL << 4))
    # TxLPFSPI::RCCAL_LPF := RCCAL
    lms_dev.reg_write_bits(0x36, (7 << 4), (RCCAL << 4))

    # Restore registers 0x05, 0x06 and 0x09
    lms_dev.reg_write(0x06, reg_save_06)
    lms_dev.reg_write(0x05, reg_save_05)
    lms_dev.reg_write(0x09, reg_save_09)

def lms_auto_calibration(lms_dev, ref_clock, lpf_bandwidth_code):
    lms_lpf_tuning_dc_calibration(lms_dev)
    lms_lpf_bandwidth_tuning(lms_dev, ref_clock, lpf_bandwidth_code)
    lms_txrx_lpf_dc_calibration(lms_dev, True)
    lms_txrx_lpf_dc_calibration(lms_dev, False)
    lms_rxvga2_dc_calibration(lms_dev)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'UmTRX LMS debugging tool.', epilog = "UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option. 'None' returned while reading\writing indicates error in the process.")
    parser.add_argument('--version', action='version', version='%(prog)s 2.4')
    parser.add_argument('--lms', type = int, choices = list(range(1, 3)), help = 'LMS number, if no other options are given it will dump all registers for corresponding LMS')
    parser.add_argument('--reg', type = lambda s: int(s, 16), choices = range(0, 0x80), metavar = '0..0x79', help = 'LMS register number, hex')
    parser.add_argument('--verify', action = 'store_true', help = 'read back written register value to verify correctness')
    parser.add_argument('--pll-ref-clock', type = float, default = 26e6, help = 'PLL reference clock, 26MHz by default')
    parser.add_argument('--lpf-bandwidth-code', type = lambda s: int(s, 16), choices = range(0, 0x10), metavar = '0..0x0f', help = 'LPF bandwidth code (default: 0x0f)')
    basic_opt = parser.add_mutually_exclusive_group()
    basic_opt.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    basic_opt.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')    
    adv_opt = parser.add_mutually_exclusive_group()
    adv_opt.add_argument('--lms-tx-enable', action = 'store_true', help = 'enable TX for LMS')
    adv_opt.add_argument('--data', type = lambda s: int(s, 16), choices = range(0, 0x100), metavar = '0..0xFF', help = 'data to be written into LMS register, hex')
    adv_opt.add_argument('--dump', action = 'store_true', help = 'dump registers')
    adv_opt.add_argument('--lms-init', action = 'store_true', help = 'run init sequence for LMS')
    adv_opt.add_argument('--lms-auto-calibration', action = 'store_true', help = 'LPF Tuning, TX/RX LPF, RXVGA2 DC Offset Calibration and LPF bandwidth tuning')
    adv_opt.add_argument('--lms-lpf-tuning-dc-calibration', action = 'store_true', help = 'DC Offset Calibration of LPF Tuning Module')
    adv_opt.add_argument('--lms-tx-lpf-dc-calibration', action = 'store_true', help = 'TX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rx-lpf-dc-calibration', action = 'store_true', help = 'RX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rxvga2-dc-calibration', action = 'store_true', help = 'RXVGA2 DC Offset Calibration')
    adv_opt.add_argument('--lms-lpf-bandwidth-tuning', action = 'store_true', help = 'LPF bandwidth tuning')
    adv_opt.add_argument('--lms-pa-on', type = int, choices = range(1, 3), help = 'turn on PA')
    adv_opt.add_argument('--lms-pa-off', action = 'store_true', help = 'turn off PA')
    adv_opt.add_argument('--lms-tx-pll-tune', type = float, metavar = '232.5e6..3720e6', help = 'Tune Tx PLL to the given frequency')
    args = parser.parse_args()
    if args.lms is None: # argparse do not have dependency concept for options
        if args.reg is not None or args.data is not None or args.lms_tx_pll_tune is not None or args.lms_init \
           or args.lms_pa_off or args.lms_pa_on is not None \
           or args.lms_lpf_tuning_dc_calibration or args.lms_tx_lpf_dc_calibration \
           or args.lms_rx_lpf_dc_calibration or args.lms_rxvga2_dc_calibration \
           or args.lms_auto_calibration or args.lms_lpf_bandwidth_tuning \
           or args.lms_tx_enable:
            exit('--lms parameter is required for given options.') # gengetopt is so much better
    if args.data is not None:
        if args.reg is None:
            exit('<data> argument requires <reg> argument.')
    if args.lms_tx_pll_tune is not None:
        if not 232.5e6 < args.lms_tx_pll_tune <= 3720e6:
            exit('<lms-tx-pll-tune> is out of range 232.5e6..3720e6')
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
            elif args.lms_tx_enable:
                lms_tx_enable(umtrx_lms_dev)
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
            elif args.lms_pa_on is not None:
                lms_pa_on(umtrx_lms_dev, args.lms_pa_on)
            elif args.lms_pa_off:
                lms_pa_off(umtrx_lms_dev)
            elif args.lms_tx_pll_tune is not None:
                lms_pll_tune(umtrx_lms_dev, int(args.pll_ref_clock), int(args.lms_tx_pll_tune))
            elif args.lms_lpf_bandwidth_tuning:
                # 0x0f - 0.75MHz
                lpf_bw_code = args.lpf_bandwidth_code if args.lpf_bandwidth_code is not None else 0x0f
                lms_lpf_bandwidth_tuning(umtrx_lms_dev, int(args.pll_ref_clock), int(lpf_bw_code))
            elif args.data is not None:
                wrt = umtrx_lms_dev.reg_write(args.reg, args.data)
                if args.verify:
                    vrfy = umtrx_lms_dev.reg_read(args.reg)
                    print('written 0x%02X to REG 0x%02X - %s' % (vrfy, args.reg, 'OK' if vrfy == args.data else 'FAIL'))
                else:
                    print('write returned 0x%02X for REG 0x%02X' % (wrt, args.reg))
            elif args.reg is not None:
                print('read 0x%02X from REG 0x%02X' % (umtrx_lms_dev.reg_read(args.reg), args.reg))
            elif args.lms:
                lms_regs = dump(umtrx_lms_dev)
                print('LMS %u' % args.lms)
                print(''.join(map(lambda a, b: '# 0x%02X: 0x%02X\n' % (a, b), list(range(0, 128)), lms_regs)))
            elif args.dump:
                umtrx_lms_dev_1 = umtrx_ctrl.umtrx_lms_device(sock, umtrx, 1)
                umtrx_lms_dev_2 = umtrx_ctrl.umtrx_lms_device(sock, umtrx, 2)
                lms1 = dump(umtrx_lms_dev_1)
                lms2 = dump(umtrx_lms_dev_2)
                diff = list(map(lambda l1, l2: 'OK\n' if l1 == l2 else 'DIFF\n', lms1, lms2))
                print(''.join(map(lambda i, l1, l2, d: '# 0x%02X: LMS1=0x%02X \tLMS2=0x%02X\t%s' % (i, l1, l2, d), list(range(0, 128)), lms1, lms2, diff)))
            else:
                print('UmTRX suspected at %s' % umtrx)
        else:
            print('UmTRX at %s is not responding.' % umtrx)
    else:
        print('No UmTRX detected over %s' % args.bcast_addr)
