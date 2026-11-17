## About

Gigabit Transceivers (GTs) are high-speed serial interfaces integrated into AMD/Xilinx FPGAs and are widely used in protocols such as PCIe, Ethernet, JESD204, and Aurora. Because signal integrity directly impacts link reliability, GTs include built-in eye scan hardware that enables the generation of eye diagrams and BER measurements, allowing engineers to evaluate channel margin, optimize equalization settings, and diagnose link issues.

## Usage

The eye scan script requires the AXI4 base address of the GTY/GTH DRP interface and provides several optional parameters to control scan resolution and range.

### Syntax

```bash
python eye_scan.py <base_addr> [options]
```

### Required Argument

| Argument | Description |
|-----------|-------------|
| `base_addr` | AXI4 base address of the GT DRP interface (for example `0xA0000000`). |

### Optional Arguments

| Argument | Default | Description |
|-----------|---------|-------------|
| `--rxout-div` | `1` | RXOUT_DIV setting. Valid values: `1`, `2`, `4`, `8`, `16`. |
| `--h-max` | `32 × RXOUT_DIV` | Maximum horizontal eye scan offset. |
| `--h-step` | `2 × RXOUT_DIV` | Horizontal scan step size. |
| `--v-max` | `120` | Maximum vertical offset (DAC codes). Scan range is `-v-max` to `+v-max`. |
| `--v-step` | `8` | Vertical scan step size. |
| `--vs-range` | `0` | RX_EYESCAN_VS_RANGE setting. Valid values: `0` to `3`. |
| `--prescale` | `4` | ES_PRESCALE value (`0–31`). Higher values increase measurement time and BER sensitivity. |
| `--bus-width` | Auto-detect | Override internal RX bus width. Valid values: `16`, `20`, `32`, `40`, `64`, `80`. |

### Examples

#### Basic Scan

```bash
python eye_scan.py 0xA0000000
```

#### Full-Rate Scan with Increased Resolution

```bash
python eye_scan.py 0xA0000000 \
    --rxout-div 1 \
    --h-step 1 \
    --v-step 4
```

#### Wide Eye Scan

```bash
python eye_scan.py 0xA0000000 \
    --h-max 64 \
    --v-max 127 \
    --prescale 6
```

### Hardware Requirements

- UltraScale+ GTY transceiver validated.
- Expected to work with UltraScale GTH transceivers with minimal or no modifications.
- GT DRP ports must be exposed to software through an AXI4-Lite to DRP bridge.
- Software must have read/write access to the bridge address space.

### Notes

- Smaller step sizes increase eye scan resolution but also increase execution time.
- Increasing `--prescale` improves BER measurement sensitivity at the cost of longer scan duration.
- If `--bus-width` is not specified, the script automatically reads the RX bus width from the transceiver DRP registers.
