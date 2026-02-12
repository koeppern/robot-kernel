# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Goal

New robotkernel-hal module for integrating a **MELECTRIC torque sensor prototype** (Serial S001) into the DLR (Deutsches Zentrum fuer Luft- und Raumfahrt) robotics HAL framework. The sensor communicates via CAN bus; raw CAN frames are received through the Beckhoff EL6751 module's **vCAN adapter option** and parsed in this module.

## Hardware Signal Chain

```
Torque Sensor (CAN) → Beckhoff EK1100 (EtherCAT Coupler) → Beckhoff EL6751 (EtherCAT-to-CAN)
                                                                      ↓
                                                              vCAN Interface (Linux)
                                                                      ↓
                                                          THIS MODULE (CAN frame parsing,
                                                           torque/sensor data extraction)
```

The EL6751 module (`module_el6751`) provides a Linux vCAN interface option. This module reads raw CAN frames from that vCAN interface and decodes the sensor protocol.

## Sensor Protocol (from prototype_doc.pdf)

### Output CAN Messages

**Torque** (500 Hz):
- CAN ID: `0x18FA8032` (extended), DLC: 8
- Payload: B0 = `0x08` | B1-B2 = Torque * 100 (signed int16) | B3-B6 = `0x00` | B7 = `0xE0`
- Torque value is scaled by 100 in both SSI and CAN

**Sensor 0-12** (100 Hz):
- CAN IDs: `0x18FA8100` through `0x18FA810C` (extended), DLC: 6
- Payload: X, Y, Z (each 16-bit signed)
- Sensors 0-7: Torque signal components
- Sensors 8-12: External field compensation

### Input CAN Message (Sensor Tare)

- Command: `0x89`, CAN ID: `0x18FA8032` (extended), DLC: 8
- Triggers sensor tare (zeroing) process

### Calibration Data

- Slope: 99,93348 Nm/Nm
- Offset: 92,565 Nm
- Linearity deviation incl. hysteresis: +0,51 % / -0,37 %
- Range: +/- 100 Nm
- Calibration date: 19.11.2025

## Upstream Framework: robotkernel-hal

GitHub Organization: https://github.com/robotkernel-hal/ (release 6.0.0, LGPL-3.0)

### Architecture Overview

Microkernel pattern with four component types:

- **Modules** (`module_*`): Hardware drivers and data processors, loaded as `.so` plugins
- **Bridges** (`bridge_*`): IPC to external apps (REST, CLI, Links-and-Nodes)
- **Service Providers** (`service_provider_*`): Acyclic RPC operations (CANopen dictionary access, etc.)
- **Core** (`robotkernel`): Module lifecycle, shared-memory process data bus, trigger engine, service framework

### Module State Machine

Every module implements: `INIT → PREOP → SAFEOP → OP` (plus BOOT and ERROR). Modeled after the EtherCAT standard. Key virtual methods:

```
set_state_init_2_preop()      // Initialize HW resources, acquire streams
set_state_preop_2_safeop()    // Acquire PD devices, register as trigger consumer
set_state_safeop_2_op()       // Enable full operation
set_state_op_2_safeop()       // Disable commands (reverse)
set_state_safeop_2_preop()    // Release PD devices (reverse)
set_state_preop_2_init()      // Release HW resources (reverse)
tick()                        // Cyclic real-time callback
```

Plugin entry point: `MODULE_DEF(module_<name>, namespace::classname)` macro.

### Process Data Naming Convention

```
<module_name>.<device>.inputs.pd       // Input process data
<module_name>.<device>.outputs.pd      // Output process data
<module_name>.<device>.inputs.trigger  // Input trigger
```

### Relevant Upstream Repositories

| Repository | Role for this project |
|---|---|
| `robotkernel` | Core framework, `module_base.h`, `trigger_base.h`, build dependency |
| `module_ethercat` | EtherCAT master managing EK1100 + EL6751 slaves |
| `module_el6751` | Beckhoff EL6751 handler providing vCAN interface for CAN frames |
| `module_pdpreprocessor` | Scaling/casting process data (for torque calibration values) |
| `module_pdrouting` | Multiplexing/demultiplexing process data between modules |
| `service_provider_canopen_protocol` | CANopen dictionary access services |

### Standard Module Repository Structure

```
module_<name>/
  src/
    <name>.h              # Class inheriting from module_base
    <name>.cpp            # Implementation (state transitions, tick)
    Makefile.am
  debian/                 # Debian packaging
  m4/                     # GNU Autotools macros
  .github/workflows/      # CI/CD
  Makefile.am
  configure.ac.in         # Autoconf template
  conanfile.py            # Conan package manager
  bootstrap.sh            # Build initialization
  module_<name>.pc.in     # pkg-config template
```

### Build Commands

```bash
./bootstrap.sh           # Generate build system (runs autoreconf)
mkdir build && cd build
../configure             # Configure
make                     # Build → produces libmodule_<name>.so
sudo make install        # Install shared library
```

### YAML Configuration Pattern

```yaml
modules:
  - name: torque_sensor
    so_file: libmodule_torque_sensor.so
    trigger: timer.cycle.trigger
    depends: [timer, el6751]
    config:
      vcan_interface: vcan0
      # sensor-specific parameters
```

## Reference Files

- Sensor protocol datasheet: `c:\temp\prototype_doc.pdf` (= `/mnt/c/temp/prototype_doc.pdf` in WSL)
