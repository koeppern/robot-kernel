# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Goal

New robotkernel-hal module for integrating a **MELECTRIC torque sensor prototype** (Serial S001) into the DLR (Deutsches Zentrum fuer Luft- und Raumfahrt) robotics HAL framework. The sensor communicates via CAN bus; raw CAN frames are received through the Beckhoff EL6751 module's **vCAN adapter option** and parsed in this module.

## Ausgelagerte Dokumentation

| Dokument | Inhalt | Trigger |
|---|---|---|
| [`docs/aufgabe.md`](docs/aufgabe.md) | Aufgabenstellung, Anforderungen, Lieferumfang, offene Entscheidungen | Bei Fragen zu Scope, Anforderungen oder Projektkontext |

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

## Implementation Specification: module_melectric_torque

### Architectural Decision: Direct SocketCAN Read

Das Modul liest **direkt** vom Linux-vCAN-Device via SocketCAN API. Es nutzt NICHT den robotkernel-Stream-Mechanismus des EL6751, sondern den Standard-Linux-Netzwerk-Stack. Die Entkopplung ist bewusst:

```
el6751 (EtherCAT PDOs) → vcan_stream.write() → Linux vCAN Device ← unser Modul liest hier
                                                Linux vCAN Device → vcan_stream.read() ← el6751 liest hier
                                                      ↑ unser Modul schreibt Tare-Command hierhin
```

Vorteil: Kein Code-Level-Dependency auf module_el6751. Nur Deployment-Dependency (el6751 muss vorher laufen und das vCAN-Device erstellen).

### Voraussetzung: vCAN-Device muss VOR dem Start existieren

Der el6751 erstellt das vCAN-Device NICHT selbst. Es muss vorher manuell oder per Systemd-Service angelegt werden:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Debugging-Tools (funktionieren direkt auf dem vCAN):
```bash
candump vcan0                          # Alle Frames mitlesen
cansend vcan0 18FA8032#8900000000000000  # Tare-Command senden
cangen vcan0                           # Test-Frames generieren
```

### Wie module_el6751 das vCAN bereitstellt (aus Quellcode-Analyse)

YAML-Config des el6751 mit vCAN:
```yaml
- name: el6751
  so_file: libmodule_el6751.so
  config:
    pd_inputs_device: ecat.slave_2.inputs.pd
    pd_outputs_device: ecat.slave_2.outputs.pd
    vcan_name: vcan0                              # <-- aktiviert vCAN
    extended_mode: true                           # Default: true (29-bit CAN IDs)
    with_padding: true                            # Default: true (2-Byte Padding pro PDO-Frame)
    tx_buf_cnt: 10                                # Default: 10 (max TX Buffer)
    rx_buf_cnt: 10                                # Default: 10 (max RX Buffer)
    slave_streams: []                             # Kann leer sein wenn nur vCAN genutzt wird
```

HINWEIS: `vcan_name`, `extended_mode`, `with_padding`, `tx_buf_cnt`, `rx_buf_cnt` sind im README NICHT dokumentiert, nur im Quellcode.

Der el6751 erstellt in `set_state_init_2_preop()` eine `vcan_stream`-Instanz, die intern ein `socket(PF_CAN, SOCK_RAW, CAN_RAW)` oeffnet, an das vCAN-Interface bindet, und CAN-Frames bidirektional zwischen EtherCAT-PDOs und dem Linux-vCAN-Device bridged.

### SocketCAN API fuer unser Modul

```cpp
// Socket oeffnen und an vCAN binden
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <poll.h>

int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
struct ifreq ifr;
strncpy(ifr.ifr_name, "vcan0", IFNAMSIZ);
ioctl(sock, SIOCGIFINDEX, &ifr);

struct sockaddr_can addr;
memset(&addr, 0, sizeof(addr));
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(sock, (struct sockaddr *)&addr, sizeof(addr));

// Non-blocking setzen fuer tick()
fcntl(sock, F_SETFL, O_NONBLOCK);

// Optional: CAN-Filter nur fuer unsere IDs
struct can_filter filters[2];
filters[0].can_id   = 0x18FA8032 | CAN_EFF_FLAG;  // Torque
filters[0].can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
filters[1].can_id   = 0x18FA8100 | CAN_EFF_FLAG;  // Sensors 0x8100-0x810C
filters[1].can_mask = 0x1FFFFF00 | CAN_EFF_FLAG;   // Maske: obere Bits matchen, untere 8 Bit variabel
setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));
```

### CAN-Frame-Parsing (Byte-Level)

**Torque-Frame** (CAN ID `0x18FA8032`, DLC 8, 500 Hz):
```
Byte:  [0]    [1]     [2]     [3]  [4]  [5]  [6]  [7]
       0x08   TQ_HI   TQ_LO   0    0    0    0    0xE0

int16_t raw_torque = (int16_t)((frame.data[1] << 8) | frame.data[2]);
// raw_torque = Torque * 100 (signed)
```

**Sensor-Frame** (CAN IDs `0x18FA8100`-`0x18FA810C`, DLC 6, 100 Hz):
```
Byte:  [0]     [1]     [2]     [3]     [4]     [5]
       X_HI    X_LO    Y_HI    Y_LO    Z_HI    Z_LO

uint32_t base_id = 0x18FA8100;
int sensor_idx = (frame.can_id & ~CAN_EFF_FLAG) - base_id;  // 0-12
int16_t x = (int16_t)((frame.data[0] << 8) | frame.data[1]);
int16_t y = (int16_t)((frame.data[2] << 8) | frame.data[3]);
int16_t z = (int16_t)((frame.data[4] << 8) | frame.data[5]);
```

**Sensor-Zuordnung**:
- Sensoren 0-7: Drehmomentsignal-Komponenten
- Sensoren 8-12: Externe Feldkompensation

**Tare-Command** (Output, an Sensor senden):
```
struct can_frame tare_frame;
tare_frame.can_id  = 0x18FA8032 | CAN_EFF_FLAG;
tare_frame.can_dlc = 8;
memset(tare_frame.data, 0, 8);
tare_frame.data[0] = 0x89;  // Tare command byte
write(sock, &tare_frame, sizeof(tare_frame));
```

### Kalibrierung: Rohwert zu Nm

Aus dem Kalibrierprotokoll (S001, 19.11.2025):
```
Lineare Regression: output = slope * reference + offset
Umkehrung:          torque_nm = (raw_can_value - offset) / slope

slope  = 99.93348   (Output-Einheiten pro Nm)
offset = 92.565     (Output bei 0 Nm, in gleichen Einheiten wie raw_can_value)

double torque_nm = ((double)raw_torque - 92.565) / 99.93348;
```

Verifikation mit Kalibrierdaten: Bei 100 Nm Referenz → CAN-Wert 10051.08
→ (10051.08 - 92.565) / 99.93348 = 99.65 Nm (Abweichung -0.17 %, innerhalb Spec)

### Process Data Output-Struktur

Das Modul exponiert einen flachen Byte-Buffer als Process Data:

```cpp
struct __attribute__((packed)) torque_sensor_output_pd {
    // Torque (aktualisiert bei 500 Hz)
    int16_t  torque_raw;           // Roh-CAN-Wert (Torque * 100)
    double   torque_nm;            // Kalibrierter Drehmomentwert in Nm

    // 13 Sensoren (aktualisiert bei 100 Hz)
    struct __attribute__((packed)) {
        int16_t x;
        int16_t y;
        int16_t z;
    } sensor[13];                  // sensor[0]-sensor[12]

    // Status
    uint8_t  torque_valid;         // 1 = kuerzlich empfangen, 0 = stale
    uint16_t sensors_valid_mask;   // Bit 0-12: je 1 Bit pro Sensor
    uint32_t torque_frame_count;   // Zaehler empfangener Torque-Frames
    uint32_t sensor_frame_count;   // Zaehler empfangener Sensor-Frames
    uint32_t error_count;          // CAN-Lesefehler
};
```

Gesamtgroesse: 2 + 8 + (13 * 6) + 1 + 2 + 4 + 4 + 4 = 103 Bytes

Process Data Device Name: `<module_name>.data.outputs.pd`

### Modul-Klasse (C++ Skeleton)

```cpp
// src/melectric_torque.h
#pragma once

#include <robotkernel/module_base.h>
#include <robotkernel/trigger_base.h>
#include <robotkernel/trigger.h>
#include <robotkernel/process_data.h>
#include <string>

namespace melectric {

class torque_sensor : public robotkernel::module_base,
                      public robotkernel::trigger_base,
                      public robotkernel::trigger {
public:
    torque_sensor(const char* name, const YAML::Node& node);
    ~torque_sensor();

    // State transitions
    void set_state_init_2_preop() override;
    void set_state_preop_2_safeop() override;
    void set_state_safeop_2_op() override;
    void set_state_op_2_safeop() override;
    void set_state_safeop_2_preop() override;
    void set_state_preop_2_init() override;

    // Cyclic callback
    void tick() override;

    // Tare service
    void tare();

private:
    // Config
    std::string vcan_interface_;
    double cal_slope_;
    double cal_offset_;
    uint32_t torque_can_id_;
    uint32_t sensor_base_can_id_;
    int sensor_count_;

    // Runtime
    int can_socket_ = -1;
    torque_sensor_output_pd pd_data_;

    // Error tracking
    uint32_t consecutive_errors_ = 0;          // Reset bei erfolgreichem read()
    static constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 100;  // → ERROR-State

    // Stale-data counters (init: stale)
    uint32_t torque_stale_counter_ = UINT32_MAX;
    uint32_t sensor_stale_counter_[13];
    static constexpr uint32_t TORQUE_STALE_THRESHOLD = 5;    // 5 ms bei 1 kHz
    static constexpr uint32_t SENSOR_STALE_THRESHOLD = 20;   // 20 ms bei 1 kHz

    // Tare thread-safety (Fallback, bis Framework-Service-Queue recherchiert)
    std::atomic<bool> tare_pending_{false};

    // Process data devices
    std::shared_ptr<robotkernel::process_data> output_pd_;
    std::shared_ptr<robotkernel::pd_provider> output_pd_provider_;

    // Helpers
    void open_can_socket();
    void close_can_socket();
    void parse_torque_frame(const struct can_frame& frame);
    void parse_sensor_frame(const struct can_frame& frame, int sensor_idx);
};

} // namespace melectric
```

### State-Transition-Implementierung

Orientiert am Muster von module_tty und module_el6751:

```
Constructor:     YAML parsen (vcan_interface, calibration, CAN IDs)
init_2_preop:    CAN-Socket oeffnen, an vCAN binden, Filter setzen
preop_2_safeop:  Output-PD-Device registrieren, Trigger-Chain binden
safeop_2_op:     Tare-Service aktivieren (Schreiben erlaubt)
tick():          drain_can_frames() → parse → PD-Buffer aktualisieren → push

op_2_safeop:     Tare-Service deaktivieren
safeop_2_preop:  Output-PD-Device deregistrieren
preop_2_init:    CAN-Socket schliessen
```

### tick() Implementierung (Kernlogik)

```cpp
void torque_sensor::tick() {
    // 1. Alle verfuegbaren CAN-Frames lesen (non-blocking)
    struct can_frame frame;
    ssize_t nbytes;
    while ((nbytes = ::read(can_socket_, &frame, sizeof(frame))) == sizeof(frame)) {
        consecutive_errors_ = 0;  // Reset bei erfolgreichem Read
        uint32_t id = frame.can_id & CAN_EFF_MASK;  // 29-bit ID ohne Flags

        if (id == torque_can_id_ && frame.can_dlc == 8) {
            parse_torque_frame(frame);
        }
        else if (id >= sensor_base_can_id_ &&
                 id < sensor_base_can_id_ + (uint32_t)sensor_count_ &&
                 frame.can_dlc == 6) {
            parse_sensor_frame(frame, id - sensor_base_can_id_);
        }
    }

    // 2. Error-Handling: EAGAIN ist normal, andere Fehler zaehlen
    if (nbytes < 0 && errno != EAGAIN) {
        pd_data_.error_count++;
        consecutive_errors_++;
        if (consecutive_errors_ >= 100) {
            // → ERROR-State ausloesen (Framework-Mechanismus)
        }
    }

    // 3. Tare-Command senden (aus Service-Queue oder atomic Flag)
    if (tare_pending_.exchange(false)) {
        struct can_frame tare_frame;
        tare_frame.can_id = torque_can_id_ | CAN_EFF_FLAG;
        tare_frame.can_dlc = 8;
        memset(tare_frame.data, 0, 8);
        tare_frame.data[0] = 0x89;
        ::write(can_socket_, &tare_frame, sizeof(tare_frame));
    }

    // 4. Stale-Data-Counter aktualisieren
    torque_stale_counter_++;
    pd_data_.torque_valid = (torque_stale_counter_ < 5) ? 1 : 0;
    for (int i = 0; i < sensor_count_; i++) {
        sensor_stale_counter_[i]++;
        if (sensor_stale_counter_[i] < 20)
            pd_data_.sensors_valid_mask |= (1 << i);
        else
            pd_data_.sensors_valid_mask &= ~(1 << i);
    }

    // 5. Process Data aktualisieren
    auto pd_ptr = output_pd_->next(output_pd_provider_);
    memcpy(pd_ptr, &pd_data_, sizeof(pd_data_));
    output_pd_->push(output_pd_provider_);
}
```

**Private Members fuer Error/Stale/Tare:**
```cpp
uint32_t consecutive_errors_ = 0;        // Reset bei erfolgreichem read()
uint32_t torque_stale_counter_ = UINT32_MAX;  // Startwert: stale
uint32_t sensor_stale_counter_[13];      // Init: alle UINT32_MAX (stale)
std::atomic<bool> tare_pending_{false};  // Fallback fuer Thread-Safety
```

### YAML-Konfiguration (komplett)

```yaml
modules:
  # Timer fuer den Echtzeit-Zyklus
  - name: timer
    so_file: libmodule_posix_timer.so
    config:
      timers:
        - name: cycle
          interval: 0.001        # 1 kHz
          mode: nanosleep

  # EtherCAT Master
  - name: ecat
    so_file: libmodule_ethercat.so
    trigger: timer.cycle.trigger
    depends: [timer]
    config:
      interface: eth0
      slaves:
        - index: 0
          name: ek1100           # Bus Coupler (passiv)
        - index: 1
          name: el6751           # CAN Gateway

  # EL6751 CAN-Bridge mit vCAN
  - name: el6751
    so_file: libmodule_el6751.so
    trigger: timer.cycle.trigger
    depends: [timer, ecat]
    config:
      pd_inputs_device: ecat.slave_1.inputs.pd
      pd_outputs_device: ecat.slave_1.outputs.pd
      vcan_name: vcan0           # Aktiviert vCAN-Adapter
      extended_mode: true        # 29-bit CAN IDs (Default)

  # UNSER MODUL: Drehmomentsensor
  - name: torque
    so_file: libmodule_melectric_torque.so
    trigger: timer.cycle.trigger
    depends: [timer, el6751]     # el6751 muss vcan0 zuerst erstellen
    config:
      vcan_interface: vcan0
      calibration:
        slope: 99.93348
        offset: 92.565
      torque_can_id: 0x18FA8032
      sensor_base_can_id: 0x18FA8100
      sensor_count: 13
```

### Build-System (Dateien die erstellt werden muessen)

```
module_melectric_torque/
  src/
    melectric_torque.h         # Header (siehe Skeleton oben)
    melectric_torque.cpp       # Implementierung
    Makefile.am                # Build-Regeln fuer src/
  Makefile.am                  # Root-Makefile
  configure.ac.in              # Autoconf-Template
  bootstrap.sh                 # Build-Initialisierung
  module_melectric_torque.pc.in  # pkg-config
  LICENSE                      # LGPL-3.0
```

**src/Makefile.am** (Muster von module_tty):
```makefile
lib_LTLIBRARIES = libmodule_melectric_torque.la
libmodule_melectric_torque_la_SOURCES = melectric_torque.cpp
libmodule_melectric_torque_la_CXXFLAGS = $(ROBOTKERNEL_CFLAGS)
libmodule_melectric_torque_la_LIBADD = $(ROBOTKERNEL_LIBS)
libmodule_melectric_torque_la_LDFLAGS = -module -avoid-version
```

**Root Makefile.am**:
```makefile
ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src
```

**module_melectric_torque.pc.in**:
```
prefix=@prefix@
exec_prefix=@exec_prefix@
libdir=@libdir@
includedir=@includedir@

Name: module_melectric_torque
Description: robotkernel module for MELECTRIC torque sensor via vCAN
Version: @VERSION@
Cflags: -I@includedir@
```

**bootstrap.sh**:
```bash
#!/bin/bash
autoreconf -if
```

### Referenz-Code aus Upstream (wichtige Patterns)

**YAML Config parsen** (aus module_tty/module_el6751):
```cpp
#include "robotkernel/helpers.h"
using robotkernel::helpers::get_as;

// Im Konstruktor:
vcan_interface_ = get_as<std::string>(node, "vcan_interface");
cal_slope_      = get_as<double>(node["calibration"], "slope", 99.93348);
cal_offset_     = get_as<double>(node["calibration"], "offset", 92.565);
```

**MODULE_DEF** (am Anfang der .cpp, nach includes/using):
```cpp
MODULE_DEF(module_melectric_torque, melectric::torque_sensor)
```

**Using-Deklarationen** (Muster aus module_tty):
```cpp
using namespace std;
using namespace robotkernel;
using namespace robotkernel::helpers;
using namespace melectric;
```

**Process Data registrieren** (aus module_el6751):
```cpp
// In set_state_preop_2_safeop():
output_pd_ = robotkernel::get_device<process_data>(name + ".data.outputs.pd");
// ODER neues PD-Device erstellen und registrieren:
robotkernel::add_device(output_pd_);
```

### Echtzeit-Constraints

- `tick()` muss deterministisch sein: keine Heap-Allokation, kein Blocking
- `::read()` mit `O_NONBLOCK` gibt `EAGAIN` zurueck wenn keine Frames → normal
- Alle CAN-Frames pro Tick drainieren (while-Loop), nicht nur einen
- Bei 1 kHz Tick-Rate: ~0-1 Torque-Frames und ~0-2 Sensor-Frames pro Tick
- `memcpy` fuer PD-Update ist deterministisch (feste Groesse)

### Stale-Data-Erkennung (entschieden: Counter-Timeout)

Counter-basiert, NICHT einfaches 0/1 pro Tick (wuerde bei 500 Hz Torque permanent flackern).

```cpp
// Private Members:
uint32_t torque_stale_counter_;      // Inkrementiert pro tick(), Reset bei Frame-Empfang
uint32_t sensor_stale_counter_[13];  // Pro Sensor, gleiche Logik

// In tick(), NACH drain_can_frames():
torque_stale_counter_++;
pd_data_.torque_valid = (torque_stale_counter_ < 5) ? 1 : 0;   // 5 ms Timeout

for (int i = 0; i < 13; i++) {
    sensor_stale_counter_[i]++;
    if (sensor_stale_counter_[i] < 20)                           // 20 ms Timeout
        pd_data_.sensors_valid_mask |= (1 << i);
    else
        pd_data_.sensors_valid_mask &= ~(1 << i);
}

// In parse_torque_frame():  torque_stale_counter_ = 0;
// In parse_sensor_frame():  sensor_stale_counter_[idx] = 0;
```

Schwellwerte: 5 Ticks (Torque, ~2.5x Periode bei 500 Hz), 20 Ticks (Sensoren, 2x Periode bei 100 Hz).

### Design-Entscheidungen (entschieden 12.02.2026)

Details und Begruendungen: siehe [`docs/aufgabe.md`](docs/aufgabe.md#design-entscheidungen)

**Brainstorming-Runde 1:**

| Entscheidung | Ergebnis |
|---|---|
| Scope V1 | Komplett in einem Zug |
| Tare-Command | robotkernel-Service (kein PD-Flag) |
| Testing | Unit-Tests mit gemockten CAN-Frames |
| Error-Handling | ERROR-State bei Socket-Fehler |
| Sensor-Kalibrierung | Nur Rohdaten (int16), kein YAGNI |
| SSI-Interface | Nicht implementieren |

**Pre-Implementation-Review (gleicher Tag):**

| Entscheidung | Ergebnis |
|---|---|
| Stale-Data-Logik | Counter-Timeout: 5 ms Torque, 20 ms Sensoren |
| Tare Thread-Safety | Framework-Service-Queue (Upstream recherchieren, Fallback: atomic Flag) |
| Lieferumfang | Nur Spec-Minimum (keine Tests, kein Packaging, kein CI) |
| Plattform | x86_64-only (double in packed Struct OK) |
| Runtime-Fehler | ERROR-State nach 100 consecutive read()-Fehlern (100 ms Toleranz) |
| CAN-Filter | Breiter Filter (Maske 0x1FFFFF00), Software-Filterung fuer unbekannte IDs |

## Reference Files

- Sensor protocol datasheet: `c:\temp\prototype_doc.pdf` (= `/mnt/c/temp/prototype_doc.pdf` in WSL)
- Upstream module_el6751 Quellcode: https://github.com/robotkernel-hal/module_el6751 (Branch: release/6.0.0)
- Upstream module_tty als Referenz-Pattern: https://github.com/robotkernel-hal/module_tty (Branch: release/6.0.0)
- Upstream robotkernel Core API: https://github.com/robotkernel-hal/robotkernel (Branch: release/6.0.0, Headers: `include/robotkernel/`)
