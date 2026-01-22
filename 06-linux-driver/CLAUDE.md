# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a university coursework project (Labor µC-Peripherie, WS 2025/26) documenting Linux peripheral device interfaces (I2C, GPIO, ADC, Timer/Counter). The documentation is written in German using Typst markup.

## Build Commands

### Document Compilation (Typst)
```bash
make compile    # Compile Typst document to PDF (dist/main.pdf)
make watch      # Watch for changes and auto-recompile
make show       # Open PDF in zathura viewer
make all        # Compile, show, and watch
```

### C Code Examples
Located in `src/figures/code/`:
```bash
cd src/figures/code
gcc 03-i2c.c -o ./out/i2c_example    # Compile specific example
./out/i2c_example                     # Run compiled example
```

Or use the Makefile (edit EXAMPLE and APP_NAME variables first):
```bash
make compile    # Compile example
make run        # Run compiled binary
```

## Architecture

### Document Structure
- `src/main.typ` - Main Typst document (465 lines)
- `src/config/text-content.yaml` - Document metadata
- `src/config/bibliography.bib` - References
- `src/figures/code/index.json` - Code snippet registry for `#render-snippet()` function

### Code Examples (src/figures/code/)
| File | Purpose |
|------|---------|
| 01-open.c | Device file opening via `open()` |
| 02-ioctl.c | ioctl configuration commands |
| 03-i2c.c | I2C read transaction with `I2C_RDWR` |
| 04-gpio.c | GPIO edge event monitoring |
| 05-adc.c | ADC temperature reading via sysfs |

Header files (030-*.h, 041-*.h, etc.) contain Linux kernel structure definitions for documentation purposes.

### Peripheral Interfaces Documented
- **I2C**: Character device `/dev/i2c-X` with ioctl
- **GPIO**: Character device `/dev/gpiochip0` with V2 line API
- **ADC**: sysfs interface `/sys/class/hwmon/hwmonX/`
- **Timer/Counter**: Data structures defined (TODO: implementation)

## Key Patterns

The project demonstrates the Linux two-interface approach:
- **sysfs (`/sys`)**: Simple text-based access (e.g., ADC)
- **devfs (`/dev`)**: Binary data and ioctl for complex operations (e.g., I2C, GPIO)

Code snippets are embedded in Typst via `#render-snippet("snippet-id")` using the index.json registry.
