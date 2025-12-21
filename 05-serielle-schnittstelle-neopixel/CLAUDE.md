# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded C project for the AT91SAM7S64 ARM microcontroller (NXT platform). The project implements I2C communication to control a GPIO expander (MCP23017) connected to LEDs and a joystick.

**Hardware Platform:**
- AT91SAM7S64 ARM7TDMI microcontroller
- NXT sensor ports for I2C communication
- MCP23017 GPIO expander on I2C bus
- 2 LEDs connected to GPIO expander (GPA0, GPA1)
- Joystick button connected to GPIO expander (GPA7)
- I2C bus on PA23 (SDA) and PA18 (SCL) with external pull-ups

## Build Commands

**Primary build command:**
```bash
make
```

The makefile configuration is controlled by the `MICRO_C_CONFIG_MARVIN` environment variable, which determines the toolchain and Trace32 paths for macOS vs Linux/WSL environments.

**Clean build:**
```bash
make clean
```

**Build modes available** (set in makefile):
- `MODE := TRACE32_RAM` - Debug mode with Trace32 debugger in RAM
- `MODE := TRACE32_ROM` - Debug mode with Trace32 debugger in ROM
- `MODE := TRACE32_SIM` - Simulation mode
- `MODE := GDBOPENOCD_RAM` - GDB with OpenOCD in RAM
- `MODE := SAMBA_RAM` - SAMBA mode in RAM

**Toolchain:**
- ARM GCC: `arm-none-eabi-gcc` (version 14.3)
- Located at: `/Users/marvin/micro-c-lab/arm-gnu-compiler/bin/` (macOS)

## Code Architecture

### Main Program Structure (main.c)

The program uses a time-sliced scheduler with 16ms cycle time:

- **Task scheduling:** Round-robin time-slice scheduler in `main()` starting at line 657
  - `task_32ms()` - Runs every 32ms
  - `task_64ms()` - Runs every 64ms
  - `task_128ms()` - Runs every 128ms (calls `io_update()`)
  - `task_256ms()` - Runs every 256ms
  - `task_512ms()` - Runs every 512ms
  - `task_1024ms()` - Runs every 1024ms
  - `task_idle()` - Runs when CPU time available

### I2C Driver Implementation (main.c:160-418)

The I2C driver is based on bit-banging (software I2C):

**Key functions to implement:**
- `I2C_delay()` - Controls I2C clock speed (main.c:235)
- `read_SCL()`, `read_SDA()` - Read pin states
- `set_SCL()`, `clear_SCL()` - Control SCL line (open-drain)
- `set_SDA()`, `clear_SDA()` - Control SDA line (open-drain)
- `arbitration_lost()` - Handle bus arbitration errors

**Protocol functions (already implemented):**
- `i2c_start_cond()` - Send I2C start condition (main.c:261)
- `i2c_stop_cond()` - Send I2C stop condition (main.c:288)
- `i2c_write_bit()` - Write single bit (main.c:314)
- `i2c_read_bit()` - Read single bit (main.c:346)
- `i2c_write_byte()` - Write byte with ACK (main.c:378)
- `i2c_read_byte()` - Read byte with ACK (main.c:401)

**GPIO Configuration:**
The I2C pins must be configured as open-drain outputs:
1. Disable internal pull-ups: `PIO_PPUDR` (external pull-ups present)
2. Enable multi-driver (open-drain): `PIO_MDER`
3. Set high (release): `PIO_SODR` - disables output driver
4. Set low (drive): `PIO_CODR` - enables output driver and pulls low

### MCP23017 GPIO Expander (main.c:423-482)

**I2C Device Address:** `0x27`

**Key registers:**
- `MCP2317_0_IODIRA/B` (0x00/0x01) - I/O direction (0=output, 1=input)
- `MCP2317_0_GPIOA/B` (0x12/0x13) - Port read/write
- `MCP2317_0_OLATA/B` (0x14/0x15) - Output latch
- `MCP2317_0_IOCONA/B` (0x0A/0x0B) - Configuration

**Pin assignments:**
- GPA0 - LED 0 (output)
- GPA1 - LED 1 (output)
- GPA6 - Joystick MUX select (output, not used in this task)
- GPA7 - Joystick button (input)

### Debugging Tools

**Oscilloscope/Logic Analyzer:**
The project includes a software oscilloscope to capture I2C signal traces:

```c
scope_init(SCOPE_SINGLE, 250);  // Capture 500 samples at 250µs intervals
scope_init(SCOPE_CONTINUE, 250); // Continuous capture
```

Display in Trace32 debugger:
```
v.draw %e scope.buf0 scope.buf1
```

**Variable watching in Trace32:**
```
Var.Watch %e %spotlight i2c_data
```

**Important:** Use slow I2C clock (`SCL_SLOW_CLOCK` defined) for initial debugging. Each bit takes 2ms (1ms delay × 2 = ~500Hz). After verification with scope, can switch to fast mode (~5kHz).

### Library Organization

Key library files in `lib/` directory:
- `aic.h/c` - Advanced Interrupt Controller
- `systick.h/c` - System timer (1ms tick)
- `term.h/c` - Terminal I/O (UART)
- `display.h/c` - NXT LCD display
- `nxt_avr.h/c` - NXT AVR coprocessor interface
- `nxt_spi.h/c` - SPI interface
- `byte_fifo_cb.h/c` - Circular buffer for FIFO

### Register Access Patterns

The code uses direct register access via pointers:
```c
AT91PS_PIO pio_a = AT91C_BASE_PIOA;  // PIO controller A base
pio_a->PIO_PPUDR = (1 << pin);        // Disable pull-up
pio_a->PIO_MDER = (1 << pin);         // Enable multi-driver
pio_a->PIO_SODR = (1 << pin);         // Set output high
pio_a->PIO_CODR = (1 << pin);         // Clear output low
pio_a->PIO_PDSR;                      // Read pin status
```

## Task Requirements (docs/task.md)

The current task requires:
1. Complete the I2C driver functions (main.c:171-181)
2. Initialize GPIO expander in `io_init()` (main.c:484)
3. Implement `io_update()` to:
   - Blink LED 0 at 1Hz (called every 128ms)
   - Mirror joystick button state to LED 1
4. Capture and document I2C bus signals using the built-in scope

## Implementation: I2C LED Blink (Completed)

### Overview
Successfully implemented I2C communication to control MCP23017 GPIO expander and blink an LED at 1Hz on GPA0.

### Critical Bug Fix: GPIO Initialization

**Problem:** Initial implementation failed because GPIO pins were not properly initialized as outputs.

**Root Cause:** The `i2c_init()` function was missing critical PIO configuration registers:
- `PIO_PER` (PIO Enable Register) - Required to transfer pin control from peripheral to PIO controller
- `PIO_OER` (Output Enable Register) - Required to enable output drivers

**Solution (main.c:183-202):**
```c
void i2c_init(void) {
  for (int lauf = 0; lauf < 4; lauf++) {
    uint32_t pin_mask = i2c_mask[lauf].i2c_scl | i2c_mask[lauf].i2c_sda;

    pio_a->PIO_PER = pin_mask;    // Enable PIO control (not peripheral)
    pio_a->PIO_OER = pin_mask;    // Enable output
    pio_a->PIO_MDER = pin_mask;   // Enable multi-driver (open-drain)
    pio_a->PIO_PPUDR = pin_mask;  // Disable internal pull-ups
    pio_a->PIO_SODR = pin_mask;   // Set idle state (high)
  }
}
```

Without these registers, pins remained in input or peripheral mode and could not drive I2C signals, causing all device scans to fail with NACK.

### MCP23017 Helper Functions (main.c:561-594)

**Device Address:** `0x27` (A2/A1/A0 address pins all HIGH)

**Write Register Function:**
```c
void mcp23017_write_reg(uint8_t reg_addr, uint8_t value) {
  i2c_write_byte(1, 0, (MCP2317_0_ADDRESS << 1) | 0); // START + Device addr + Write
  i2c_write_byte(0, 0, reg_addr);                     // Register address
  i2c_write_byte(0, 1, value);                        // Data + STOP
}
```

**Read Register Function:**
```c
uint8_t mcp23017_read_reg(uint8_t reg_addr) {
  i2c_write_byte(1, 0, (MCP2317_0_ADDRESS << 1) | 0); // START + Device addr + Write
  i2c_write_byte(0, 0, reg_addr);                     // Register address
  i2c_write_byte(1, 0, (MCP2317_0_ADDRESS << 1) | 1); // RESTART + Device addr + Read
  uint8_t data = i2c_read_byte(1, 1);                 // Read with NACK + STOP
  return data;
}
```

**Key I2C Protocol Details:**
- Parameter 1 of `i2c_write_byte()`: `1` = send START/RESTART, `0` = continue
- Parameter 2 of `i2c_write_byte()`: `1` = send STOP, `0` = continue
- Parameter 1 of `i2c_read_byte()`: `1` = send NACK (last byte), `0` = send ACK
- Parameter 2 of `i2c_read_byte()`: `1` = send STOP, `0` = continue

### Initialization Sequence (main.c:596-649)

**io_init() performs:**
1. **I2C Scanner** (optional debug): Scans addresses 0x20-0x27 to find MCP23017
2. **Configure IODIRA** (0x00): Set all Port A pins as outputs (write 0x00)
3. **Write OLATA** (0x14): Initialize output latch to 0xFF (all outputs HIGH)
4. **Verify Communication**: Read back IODIRA and OLATA registers
5. **Scope Capture**: Initialize software oscilloscope for signal analysis

**Results stored in i2c_data for Trace32 debugging:**
- `i2c_data.mcp_iodira_readback` - Should read 0x00 if device responds
- `i2c_data.mcp_olata_readback` - Should read 0xFF after initialization
- `i2c_data.scan_results[8]` - I2C scanner results (1 = ACK, 0 = NACK)
- `i2c_data.scan_complete` - Set to 1 when scan finishes

### LED Blink Implementation (main.c:658-673)

**io_update() called every 128ms from task_128ms():**

```c
void io_update(void) {
  static uint8_t blink_counter = 0;
  static uint8_t led0_state = 0;

  blink_counter++;
  if (blink_counter >= 4) {  // 128ms × 4 = 512ms toggle period
    blink_counter = 0;
    led0_state ^= 1;         // Toggle LED state
  }

  mcp23017_write_reg(MCP2317_0_OLATA, led0_state);  // Write to output latch
}
```

**Timing Calculation:**
- io_update() runs every 128ms
- Counter increments: 0 → 1 → 2 → 3 → 4 (reset)
- Toggle period: 128ms × 4 = 512ms
- Full cycle: 512ms × 2 = 1024ms ≈ **1Hz blink rate**

**Why OLATA not GPIOA:**
- `OLATA` (0x14): Output latch register - holds output state for pins configured as outputs
- `GPIOA` (0x12): Port register - reads actual pin states (useful for inputs)
- Writing to OLATA is the correct method for controlling outputs per MCP23017 datasheet

### I2C Scanner Tool (main.c:530-559)

Added diagnostic scanner to detect I2C devices on the bus:

```c
void i2c_scan_mcp23017(void) {
  for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
    bool nack = i2c_write_byte(1, 1, (addr << 1) | 0);  // Try address with START+STOP
    i2c_data.scan_results[addr - 0x20] = (nack == 0) ? 1 : 0;  // Store ACK/NACK
  }
}
```

Scanner sends only device address byte. Devices respond with ACK (pull SDA low) if present at that address.

### Debug Data Structure (main.c:167-179)

Extended i2c_data struct with debugging fields:

```c
struct {
  struct i2c { int started; int arbitration_lost; } i2c[4];
  uint8_t mcp_iodira_readback;    // IODIRA register readback value
  uint8_t mcp_olata_readback;     // OLATA register readback value
  uint8_t mcp_init_complete;      // Initialization complete flag
  uint8_t scan_results[8];        // I2C scan results (0x20-0x27)
  uint8_t scan_complete;          // Scan complete flag
} i2c_data;
```

**Trace32 Monitoring:**
```
Var.Watch %e %spotlight i2c_data
```

### Verification Results

**Working I2C Communication:**
- `i2c_data.scan_results[7] = 1` - Device found at address 0x27 ✓
- `i2c_data.mcp_iodira_readback = 0x00` - Direction register configured correctly ✓
- `i2c_data.mcp_olata_readback = 0xFF` - Output latch set correctly ✓
- `i2c_data.i2c[0].arbitration_lost = 0` - No bus errors ✓

**Hardware Status:**
- LED on GPA0 blinking at ~1Hz ✓
- Additional LED on (from initial 0xFF write to OLATA)

### I2C Timing (Slow Clock Mode)

With `SCL_SLOW_CLOCK` defined:
- Bit time: ~2ms (1ms delay each edge)
- Byte time: ~18ms (8 data bits + 1 ACK bit × 2ms)
- Register write: ~54ms (3 bytes: address, register, data)
- Register read: ~72ms (write sequence + restart + read sequence)
- Well within 128ms task period

### Key Learnings

1. **GPIO Configuration is Critical**: AT91SAM7S requires explicit PIO_PER and PIO_OER configuration before pins can drive signals
2. **I2C Address Detection**: Use scanner to verify device presence before assuming address
3. **Open-Drain Configuration**: Requires PIO_MDER for multi-driver mode and external pull-ups
4. **Register Choice Matters**: Always write to OLATA for outputs, read from GPIOA for inputs
5. **Debugging Strategy**: Use readback verification and scan results stored in watchable struct

## Important Notes

- **Cycle time:** Tasks run on 16ms scheduler, but timing violations are currently disabled (see main.c:685)
- **I2C timing:** Each byte transmission takes ~18ms with slow clock, ~1.8ms with fast clock
- **Stack monitoring:** Automatic stack overflow detection enabled (main.c:698)
- **Battery monitoring:** Low battery detection at 6V threshold (main.c:713)
- **Watchdog:** Disabled in current configuration (main.c:643)
- **Interrupt context:** PWM interrupt handler `pwm_isr_entry()` samples I2C lines for scope (main.c:97)

## VT100 Terminal Codes

The project uses VT100 escape sequences for terminal output (defined in main.h):
- Cursor control: `VT100_CURSORHOME`, `VT100_CLEARSCREEN`
- Colors: `VT100_VORDERGRUND_ROT`, `VT100_VORDERGRUND_GRUEN`, etc.
- Formatting: `VT100_FETT`, `VT100_UNTERSTRICHEN`, `VT100_BLINKEN`

Terminal output uses `term_string()` for async/sync writes or `printf()`/`iprintf()` for formatted output.
