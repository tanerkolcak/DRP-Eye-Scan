#!/usr/bin/env python3
"""
    UltraScale+ GTY (GTYE4_CHANNEL) Manual Eye Scan

    References:
        UG578, v1.3.1, September 14, 2021
        AMD AR 68785, Manual Eye Scan with UltraScale+ GTY

    Example usage
        sudo python3 drp_eye_scan.py 0xA0050000 --rxout-div 1 --dfe (full rate with DFE)
        sudo python3 drp_eye_scan.py 0xA0050000 --rxout-div 1 (full rate with LPM)
        sudo python3 drp_eye_scan.py 0xA0050000 --rxout-div 2 --dfe (half rate with DFE)
        sudo python3 drp_eye_scan.py 0xA0050000 --rxout-div 2 (half rate with LPM)

    Author
        Taner Kolcak
"""

import argparse
import math
import mmap
import os
import struct
import sys
import time

# ==============================================================================
# GTYE4_CHANNEL DRP Register Map
# ==============================================================================
DRP_RX_DATA_WIDTH       = 0x003  # [ 8: 5] : User side parallel data width)
                                 #       2:  16
                                 #       3:  20
                                 #       4:  32
                                 #       5:  40
                                 #       6:  64
                                 #       7:  80
                                 #       8:  128
                                 #       9:  190
DRP_RX_INT_DATAWIDTH    = 0x066  # [ 1: 0] (Internal PMA to PCS width used by Eye Scan)
                                 #       0 : 2 bytes (16 or 20 bits)
                                 #       1 : 4 bytes (32 or 40 bits)
                                 #       2 : 8 bytes (64 or 80 bits)
DRP_ES_CONTROL          = 0x03C  # [15:10] ES_CONTROL
                                 #     [15:12] ARMED state triggers controling the state transition to READ state (check UG578 Table 4-20 for details)
                                 #     [11:11] ARM : Asserting this bit causes a state transition from the WAIT state to the RESET state, initiating a diagnostic sequence.
                                 #     [10:10] RUN : Asserting this bit causes a state transition from the WAIT state to the RESET state, initiating a BER measurement sequence.
                                 # [ 9: 9] ES_ERRDET_EN
                                 # [ 8: 8] ES_EYE_SCAN_EN
                                 # [ 4: 0] ES_PRESCALE
DRP_ES_SDATA_MASK_LO    = (0x049, 0x04A, 0x04B, 0x04C, 0x04D)  # (ES_SDATA_MASK0, .., ES_SDATA_MASK4)
DRP_ES_SDATA_MASK_HI    = (0x0F1, 0x0F2, 0x0F3, 0x0F4, 0x0F5)  # (ES_SDATA_MASK5, .., ES_SDATA_MASK9)
DRP_ES_QUAL_MASK_LO     = (0x044, 0x045, 0x046, 0x047, 0x048)  # (ES_QUAL_MASK0, .., ES_QUAL_MASK4)
DRP_ES_QUAL_MASK_HI     = (0x0EC, 0x0ED, 0x0EE, 0x0EF, 0x0F0)  # (ES_QUAL_MASK5, .., ES_QUAL_MASK9)
DRP_ES_HORZ_OFFSET      = 0x04F  # [15: 4] : ES_HORZ_OFFSET
                                 #     [15:15] : Phase unification
                                 #     [14: 4] : Phase offset (two's complement)
DRP_RX_EYESCAN_VS       = 0x097  # [10:10] : RX_EYESCAN_VS_NEG_DIR
                                 # [ 9: 9] : RX_EYESCAN_VS_UT_SIGN
                                 # [ 8: 2] : RX_EYESCAN_VS_CODE
                                 # [ 1: 0] : RX_EYESCAN_VS_RANGE
DRP_RX_EYESCAN_VS_MASK  = 0x07FF

# Read-only status & counter registers (10-bit DRP addresses, DRPADDR=1)
DRP_ES_ERROR_COUNT      = 0x251  # [15: 0] es_error_count
DRP_ES_SAMPLE_COUNT     = 0x252  # [15: 0] es_sample_count
DRP_ES_CTRL_STATUS      = 0x253  # [ 3: 1] Current state of the state machine
                                 #     WAIT  : 000
                                 #     RESET : 001
                                 #     COUN  : 011
                                 #     END   : 010
                                 #     ARMED : 101
                                 #     READ  : 100
                                 # [ 0: 0] DONE (asserted high only in WAIR, END or READ states)

ES_FSM_STATE_WAIT       = 0b000
ES_FSM_STATE_END        = 0b010

# Approximate vertical DAC step size in mV per code for GTYE4 (UG578)
ES_VS_RANGE_MV_PER_CODE = {0: 1.5, 1: 1.8, 2: 2.2, 3: 2.8}

class AXIDRPInterface:
    """ Direct word-aligned AXI4-Lite to GTYE4 DRP bridge via /dev/mem """

    def __init__(self, base_addr: int, map_size: int = 0x10000):
        self.base_addr = base_addr
        self.map_size = map_size
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(
            self.fd,
            self.map_size,
            mmap.MAP_SHARED,
            mmap.PROT_READ | mmap.PROT_WRITE,
            offset=self.base_addr,
        )

    def drp_read(self, drp_addr: int) -> int:
        """ Read a 16-bit DRP register at AXI_BASE + (drp_addr << 2) """
        byte_offset = (drp_addr & 0x3FF) << 2
        val = struct.unpack("<I", self.mem[byte_offset : byte_offset + 4])[0]
        return val & 0xFFFF

    def drp_write(self, drp_addr: int, value: int) -> None:
        """ Write a 16-bit value to a DRP register at AXI_BASE + (drp_addr << 2) """
        byte_offset = (drp_addr & 0x3FF) << 2
        self.mem[byte_offset : byte_offset + 4] = struct.pack("<I", value & 0xFFFF)

    def drp_rmw(self, drp_addr: int, mask: int, value: int) -> None:
        """ Read-Modify-Write helper preserving unmasked DRP bits """
        curr = self.drp_read(drp_addr)
        updated = (curr & ~mask) | (value & mask)
        self.drp_write(drp_addr, updated)

    def close(self):
        self.mem.close()
        os.close(self.fd)

class GTYE4ManualEyeScan:
    """ GTYE4_CHANNEL manual eye ccan controller """

    def __init__(
        self,
        drp: AXIDRPInterface,
        bus_width: int = None,
        prescale: int = 4,
        dfe_mode: bool = True,
        vs_range: int = 0,
        phase_unify_bit: int = 1,
    ):
        self.drp = drp
        self.bus_width = bus_width if bus_width is not None else self.detect_bus_width()
        self.prescale = prescale
        self.dfe_mode = dfe_mode
        self.vs_range = vs_range & 0x3
        self.phase_unify_bit = phase_unify_bit & 0x1

    def detect_bus_width(self) -> int:
        """ Read DRP_RX_DATA_WIDTH and DRP_RX_INT_DATAWIDTH to determine internal PMA to PCS bus width """
        reg003 = self.drp.drp_read(DRP_RX_DATA_WIDTH)
        rx_dw_enc = (reg003 >> 5) & 0x7
        reg066 = self.drp.drp_read(DRP_RX_INT_DATAWIDTH)
        rx_int_dw = reg066 & 0x3

        if rx_dw_enc not in (2, 3, 4, 5, 6, 7) or rx_int_dw not in (0, 1, 2):
            print(
                f"[!] Warning: Unexpected DRP DRP_RX_DATA_WIDTH value (0x{reg03:04X}). Using default bus_width=40.",
                file=sys.stderr,
            )
            return 40

        is_10b = rx_dw_enc in (3, 5, 7)
        detected = (20 if is_10b else 16) * (1 << rx_int_dw)
        print(
            f"[*] Auto-detected internal bus width = {detected}-bit "
            f"(DRP RX_DATA_WIDTH=0x{reg003:04X}, RX_DATA_WIDTH(ENCODED)={rx_dw_enc}, RX_INT_DATAWIDTH={rx_int_dw})"
        )
        return detected

    def initialize(self):
        """ Enable Eye Scan, set Prescale, configure 160-bit masks and zero offsets """
		# Set ES_CONTROL[5:0]=000000 (WAIT), ES_ERRDET_EN=1, ES_EYE_SCAN_EN=1 and ES_PRESCALE
        ctrl_val = (
            (0x00 << 10)
            | (1 << 9)
            | (1 << 8)
            | (self.prescale & 0x1F)
        )
        self.drp.drp_rmw(DRP_ES_CONTROL, 0xFF1F, ctrl_val)

		# Set ES_QUAL_MASK[159:0] = {160{1'b1}}
        for addr in DRP_ES_QUAL_MASK_LO + DRP_ES_QUAL_MASK_HI:
            self.drp.drp_write(addr, 0xFFFF)

		# Set ES_SDATA_MASK[159:0] (unmask active bus_width bits (0) and mask rest (1))
        lower_80_active = ((1 << self.bus_width) - 1) << (80 - self.bus_width)
        lower_80_mask = ((1 << 80) - 1) ^ lower_80_active
        sdata_mask_160b = (((1 << 80) - 1) << 80) | lower_80_mask

        all_sdata_addrs = DRP_ES_SDATA_MASK_LO + DRP_ES_SDATA_MASK_HI
        for idx, addr in enumerate(all_sdata_addrs):
            word = (sdata_mask_160b >> (idx * 16)) & 0xFFFF
            self.drp.drp_write(addr, word)

        self.set_horizontal_offset(0)
        self.set_vertical_offset(0, ut_sign=0)

    def set_horizontal_offset(self, horz_offset: int):
        """ Configure 12-bit ES_HORZ_OFFSET """
        offset_11b = horz_offset & 0x07FF
        es_horz_12b = (self.phase_unify_bit << 11) | offset_11b
        self.drp.drp_rmw(DRP_ES_HORZ_OFFSET, 0xFFF0, (es_horz_12b << 4))

    def set_vertical_offset(self, vert_offset: int, ut_sign: int = 0):
        """
        Configure vertical offset
            [10:10] : RX_EYESCAN_VS_NEG_DIR
            [ 9: 9] : RX_EYESCAN_VS_UT_SIGN
            [ 8: 2] : RX_EYESCAN_VS_CODE[6:0]
            [ 1: 0] : RX_EYESCAN_VS_RANGE[1:0]
        """
        neg_dir = 1 if vert_offset < 0 else 0
        vs_code = min(abs(vert_offset), 127) & 0x7F
        val = (
            ( (neg_dir & 0x1) << 10)
            | ((ut_sign & 0x1) << 9)
            | (vs_code << 2)
            | (self.vs_range & 0x3)
        )
        self.drp.drp_rmw(DRP_RX_EYESCAN_VS, DRP_RX_EYESCAN_VS_MASK, val)

    def measure_single_ut(self, timeout_s: float = 0.5) -> tuple:
        """ Trigger FSM RUN, poll DRP_ES_CTRL_STATUS for END state (3'b010), read counters and return to WAIT """
        self.drp.drp_rmw(DRP_ES_CONTROL, 0xFC00, (0x01 << 10))

        t_start = time.monotonic()
        while True:
            status = self.drp.drp_read(DRP_ES_CTRL_STATUS)
            fsm_state = (status >> 1) & 0x7
            done_bit = status & 0x1
            if fsm_state == ES_FSM_STATE_END or done_bit == 1:
                break
            if (time.monotonic() - t_start) > timeout_s:
                break

        err_cnt = self.drp.drp_read(DRP_ES_ERROR_COUNT)
        smp_cnt = self.drp.drp_read(DRP_ES_SAMPLE_COUNT)

        self.drp.drp_rmw(DRP_ES_CONTROL, 0xFC00, (0x00 << 10))
        return err_cnt, smp_cnt

    def measure_point_ber(self, h_off: int, v_off: int) -> tuple:
        """ Return (ber, total_bits_sampled) at (h_off, v_off) (take average of ±UT in DFE mode) """
        self.set_horizontal_offset(h_off)
        ut_list = (0, 1) if self.dfe_mode else (0,)
        total_errors = 0
        total_bits = 0.0

        for ut in ut_list:
            self.set_vertical_offset(v_off, ut_sign=ut)
            err, smp = self.measure_single_ut()
            bits = max(smp, 1) * self.bus_width * (2 ** (1 + self.prescale))
            total_errors += err
            total_bits += bits

        ber = 0.0 if total_errors == 0 else min(1.0, total_errors / total_bits)
        return ber, total_bits

def compute_eye_metrics(
    ber_grid: list,
    h_offsets: list,
    v_offsets: list,
    rxout_div: int,
    vs_range: int,
    ber_threshold: float = 0.0,
) -> dict:
    """ Calculate horizontal eye opening (UI), vertical eye opening (codes & mV) and open area """
    steps_per_ui = 64.0 * rxout_div
    mv_per_code = ES_VS_RANGE_MV_PER_CODE.get(vs_range, 1.5)

    h_zero_idx = min(range(len(h_offsets)), key=lambda i: abs(h_offsets[i]))
    v_zero_idx = min(range(len(v_offsets)), key=lambda i: abs(v_offsets[i]))

    # Horizontal slice at V = 0
    h_row = ber_grid[v_zero_idx]
    left_idx = h_zero_idx
    while left_idx > 0 and h_row[left_idx - 1] <= ber_threshold:
        left_idx -= 1

    right_idx = h_zero_idx
    while right_idx < len(h_offsets) - 1 and h_row[right_idx + 1] <= ber_threshold:
        right_idx += 1

    if h_row[h_zero_idx] <= ber_threshold:
        eye_width_steps = h_offsets[right_idx] - h_offsets[left_idx]
        h_left_val = h_offsets[left_idx]
        h_right_val = h_offsets[right_idx]
    else:
        eye_width_steps = 0
        h_left_val = 0
        h_right_val = 0

    eye_width_ui = eye_width_steps / steps_per_ui

    # Vertical slice at H = 0 (v_offsets is ordered descending +V to -V)
    top_idx = v_zero_idx
    while top_idx > 0 and ber_grid[top_idx - 1][h_zero_idx] <= ber_threshold:
        top_idx -= 1

    bot_idx = v_zero_idx
    while bot_idx < len(v_offsets) - 1 and ber_grid[bot_idx + 1][h_zero_idx] <= ber_threshold:
        bot_idx += 1

    if ber_grid[v_zero_idx][h_zero_idx] <= ber_threshold:
        eye_height_codes = v_offsets[top_idx] - v_offsets[bot_idx]
        v_top_val = v_offsets[top_idx]
        v_bot_val = v_offsets[bot_idx]
    else:
        eye_height_codes = 0
        v_top_val = 0
        v_bot_val = 0

    eye_height_mv = eye_height_codes * mv_per_code

    total_cells = len(v_offsets) * len(h_offsets)
    open_cells = sum(1 for row in ber_grid for val in row if val <= ber_threshold)
    open_pct = (open_cells / total_cells) * 100.0 if total_cells else 0.0

    return {
        "center_ber": ber_grid[v_zero_idx][h_zero_idx],
        "eye_width_steps": eye_width_steps,
        "eye_width_ui": eye_width_ui,
        "h_left": h_left_val,
        "h_right": h_right_val,
        "eye_height_codes": eye_height_codes,
        "eye_height_mv": eye_height_mv,
        "v_top": v_top_val,
        "v_bot": v_bot_val,
        "open_cells": open_cells,
        "total_cells": total_cells,
        "open_pct": open_pct,
    }


def ber_to_ansi_cell(ber: float) -> str:
    """ Map BER value to an ANSI 256-color 2-character ASCII block """
    if ber == 0.0:
        return "\033[48;5;17m\033[38;5;27m  \033[0m"     # Deep Blue (Open Eye / 0 errors)
    log_ber = math.log10(ber)
    if log_ber <= -9.0:
        return "\033[48;5;21m\033[38;5;39m..\033[0m"     # Blue (<= 1e-9)
    elif log_ber <= -7.0:
        return "\033[48;5;37m\033[38;5;51m::\033[0m"     # Cyan (<= 1e-7)
    elif log_ber <= -5.0:
        return "\033[48;5;34m\033[38;5;46m--\033[0m"     # Green (<= 1e-5)
    elif log_ber <= -3.0:
        return "\033[48;5;220m\033[38;5;16m==\033[0m"    # Yellow (<= 1e-3)
    elif log_ber <= -1.5:
        return "\033[48;5;208m\033[38;5;16m++\033[0m"    # Orange (<= 10^-1.5)
    else:
        return "\033[48;5;196m\033[38;5;15m##\033[0m"    # Red (High BER / Eye Edge)

def plot_ascii_eye_and_metrics(
    ber_grid: list,
    h_offsets: list,
    v_offsets: list,
    metrics: dict,
    ber_floor: float,
    eq_mode: str,
    rxout_div: int,
    bus_width: int,
):
    """ Render the 2D BER matrix and summary metrics in the terminal """
    print("\n" + "=" * 78)
    print(f"  ULTRASCALE+ GTYE4 EYE DIAGRAM ({eq_mode} | RXOUT_DIV={rxout_div} | {bus_width}-bit)")
    print("=" * 78)

    for v_idx, v in enumerate(v_offsets):
        row_str = f"{v:+4d} |"
        for h_idx in range(len(h_offsets)):
            row_str += ber_to_ansi_cell(ber_grid[v_idx][h_idx])
        print(row_str)

    print("     +" + "--" * len(h_offsets))

    tick_line = "      "
    for h_idx, h in enumerate(h_offsets):
        if h_idx % 4 == 0:
            tick_line += f"{h:<8d}"
    print(tick_line)
    print(" " * 28 + "Horizontal Offset (UI steps)\n")

    legend = (
        "Legend: "
        f"{ber_to_ansi_cell(0.0)} 0 Err  "
        f"{ber_to_ansi_cell(1e-9)} <=1e-9  "
        f"{ber_to_ansi_cell(1e-7)} <=1e-7  "
        f"{ber_to_ansi_cell(1e-5)} <=1e-5  "
        f"{ber_to_ansi_cell(1e-3)} <=1e-3  "
        f"{ber_to_ansi_cell(1e-2)} <=1e-2  "
        f"{ber_to_ansi_cell(0.5)} >1e-1"
    )
    print(legend)

    print("-" * 78)
    print("  EYE OPENING METRICS (0-Error Contour)")
    print("-" * 78)
    print(f"  Center BER (0, 0)       : {metrics['center_ber']:.2e} (BER floor ~ {ber_floor:.2e})")
    print(
        f"  Horizontal Eye Width    : {metrics['eye_width_ui']:.3f} UI "
        f"({metrics['eye_width_steps']} steps, [{metrics['h_left']:+d} .. {metrics['h_right']:+d}])"
    )
    print(
        f"  Vertical Eye Height     : {metrics['eye_height_codes']} DAC codes "
        f"(~{metrics['eye_height_mv']:.1f} mV, [{metrics['v_bot']:+d} .. {metrics['v_top']:+d}])"
    )
    print(
        f"  Open Eye Area           : {metrics['open_cells']}/{metrics['total_cells']} grid points "
        f"({metrics['open_pct']:.1f}%)"
    )
    print("=" * 78 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="UltraScale+ GTYE4_CHANNEL Manual Eye Scan (UG578 / AR 68785) over AXI4 DRP"
    )
    parser.add_argument(
        "base_addr",
        type=lambda x: int(x, 0),
        help="AXI4 Base Address of the GTYE4 DRP port (e.g., 0xA0000000)",
    )
    parser.add_argument("--rxout-div", type=int, default=1, choices=(1, 2, 4, 8, 16), help="RXOUT_DIV (1=Full-Rate, default: 1)")
    parser.add_argument("--h-max", type=int, default=None, help="Max horizontal offset (default: 32 * rxout_div)")
    parser.add_argument("--h-step", type=int, default=None, help="Horizontal step size (default: 2 * rxout_div)")
    parser.add_argument("--v-max", type=int, default=120, help="Max vertical offset (+/- DAC codes, default: 120)")
    parser.add_argument("--v-step", type=int, default=8, help="Vertical step size (default: 8)")
    parser.add_argument("--vs-range", type=int, default=0, choices=(0, 1, 2, 3), help="RX_EYESCAN_VS_RANGE [0..3] (default: 0)")
    parser.add_argument("--prescale", type=int, default=4, help="ES_PRESCALE [0..31] (default: 4)")
    parser.add_argument(
        "--bus-width",
        type=int,
        default=None,
        choices=(16, 20, 32, 40, 64, 80),
        help="Override internal bus width in bits (default: auto-detect from DRP RX_DATA_WIDTH)",
    )
    parser.add_argument("--dfe", action="store_true", help="Enable DFE mode (averages +UT and -UT samplers)")
    parser.add_argument("--phase-bit", type=int, default=1, choices=(0, 1), help="ES_HORZ_OFFSET phase unification bit (default: 1)")
    args = parser.parse_args()

    h_max = args.h_max if args.h_max is not None else (32 * args.rxout_div)
    h_step = args.h_step if args.h_step is not None else (2 * args.rxout_div)

    h_offsets = list(range(-h_max, h_max + 1, h_step))
    v_offsets = list(range(args.v_max, -args.v_max - 1, -args.v_step))

    print(f"[*] Opening AXI4 DRP interface at base address 0x{args.base_addr:08X}...")
    try:
        drp = AXIDRPInterface(args.base_addr)
    except PermissionError:
        print("[!] Permission denied opening /dev/mem. Run with sudo/root.", file=sys.stderr)
        sys.exit(1)

    scanner = GTYE4ManualEyeScan(
        drp=drp,
        bus_width=args.bus_width,
        prescale=args.prescale,
        dfe_mode=args.dfe,
        vs_range=args.vs_range,
        phase_unify_bit=args.phase_bit,
    )

    try:
        print("[*] Initializing GTYE4 Eye Scan registers ...")
        scanner.initialize()

        center_ber, sample_bits = scanner.measure_point_ber(0, 0)
        ber_floor = 1.0 / max(sample_bits, 1.0)
        if center_ber > 0.0:
            print(
                f"[!] Warning: Non-zero BER ({center_ber:.2e}) at center (0,0). "
                "Verify DRP address bus connection, bus width or EYESCANRESET.",
                file=sys.stderr,
            )

        total_pts = len(v_offsets) * len(h_offsets)
        done_pts = 0
        ber_grid = []

        eq_label = "DFE" if args.dfe else "LPM"
        print(f"[*] Sweeping {len(h_offsets)}x{len(v_offsets)} ({total_pts} pts) | {eq_label} | RXOUT_DIV={args.rxout_div}...")
        for v in v_offsets:
            row = []
            for h in h_offsets:
                ber, _ = scanner.measure_point_ber(h, v)
                row.append(ber)
                done_pts += 1
            ber_grid.append(row)
            pct = (done_pts / total_pts) * 100
            print(f"\r    Progress: {done_pts}/{total_pts} ({pct:5.1f}%)", end="", flush=True)

        print()
        metrics = compute_eye_metrics(
            ber_grid=ber_grid,
            h_offsets=h_offsets,
            v_offsets=v_offsets,
            rxout_div=args.rxout_div,
            vs_range=args.vs_range,
            ber_threshold=0.0,
        )
        plot_ascii_eye_and_metrics(
            ber_grid=ber_grid,
            h_offsets=h_offsets,
            v_offsets=v_offsets,
            metrics=metrics,
            ber_floor=ber_floor,
            eq_mode=eq_label,
            rxout_div=args.rxout_div,
            bus_width=scanner.bus_width,
        )
    finally:
        drp.close()

if __name__ == "__main__":
    main()
