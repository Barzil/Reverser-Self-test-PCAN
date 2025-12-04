# Reverser Diagnostics

PCAN-based diagnostics tool for reverser board testing.

## Requirements

- Python 3.6 or higher
- PCAN-Basic library (DLL) installed on your system
  - Download from: https://www.peak-system.com/PCAN-Basic.239.0.html
  - The DLL should be accessible to Python (typically in system PATH or same directory)

## Installation

1. Ensure PCAN-Basic DLL is installed on your system
2. No additional Python packages are required (uses only standard library)

## Usage

Run the diagnostics:

```bash
python -m reverser_diagnostics.test_reverser
```

Or from the project directory:

```bash
cd reverser_diagnostics_openhtf_full_v2
python -m reverser_diagnostics.test_reverser
```

## Project Structure

- `reverser_diagnostics/` - Main package
  - `pcan_reverser.py` - Core diagnostics logic
  - `test_reverser.py` - Entry point and reporting
- `PCANBasic.py` - PCAN-Basic API wrapper

## Diagnostics Steps

The tool runs 6 diagnostic steps:

1. **Connectivity** - Verifies CAN bus communication
2. **Configuration** - Checks test station configuration (active_mask)
3. **Manual** - Tests manual mode (all channels ~2.5V)
4. **Operational** - Tests operational baseline mode
5. **Forward** - Tests forward gear (all channels ON)
6. **Reverse** - Tests reverse gear (all channels OFF)

## Output

The tool provides:
- Step-by-step PASS/FAIL summary
- Per-channel diagnostics table showing voltage readings
- Final overall result

