# module_melectric_torque

robotkernel-hal Modul fuer den **MELECTRIC Drehmomentsensor-Prototyp** (S001). Liest CAN-Frames ueber ein Linux-vCAN-Interface und stellt Drehmoment- und Magnetfelddaten als Process Data bereit.

## Hardware-Signalkette

```
Drehmomentsensor (CAN-Bus)
    → Beckhoff EL6751 (EtherCAT-to-CAN Gateway)
        → Beckhoff EK1100 (EtherCAT Bus Coupler)
            → module_el6751 (Software-Treiber)
                → Linux vCAN Interface (vcan0)
                    → DIESES MODUL
```

Das Modul hat keine Code-Abhaengigkeit auf `module_el6751`. Es liest direkt vom Linux-vCAN-Device ueber die Standard-SocketCAN-API.

## Was das Modul macht

- **Torque-Dekodierung** (500 Hz): CAN ID `0x18FA8032`, int16 aus Bytes 1-2, Kalibrierung in Nm
- **Sensor-Dekodierung** (100 Hz): CAN IDs `0x18FA8100`-`0x18FA810C`, 13 Sensoren mit je X/Y/Z (int16)
- **Stale-Data-Erkennung**: Counter-basiert (5 ms Torque, 20 ms Sensoren)
- **Tare-Service**: Nullpunkt-Kalibrierung ueber robotkernel-Service-Mechanismus
- **Process Data**: 103-Byte Output-Buffer im Shared-Memory-Bus

## Dateien

```
module_melectric_torque/
  src/
    melectric_torque.h         # Klassen-Header mit PD-Struct (103 Bytes)
    melectric_torque.cpp       # Implementierung (~235 Zeilen)
    Makefile.am
  Makefile.am
  configure.ac.in              # Autoconf-Template (C++14, robotkernel pkg-config)
  bootstrap.sh                 # Generiert configure.ac aus Template, ruft autoreconf auf
  module_melectric_torque.pc.in
  LICENSE                      # LGPL-3.0
```

## Bauen

Voraussetzung: robotkernel 6.0.0 muss installiert sein (Headers + pkg-config).

```bash
cd module_melectric_torque
./bootstrap.sh
mkdir build && cd build
../configure
make
sudo make install    # Installiert libmodule_melectric_torque.so
```

## YAML-Konfiguration

```yaml
modules:
  - name: torque
    so_file: libmodule_melectric_torque.so
    trigger: timer.cycle.trigger
    depends: [timer, el6751]
    config:
      vcan_interface: vcan0
      calibration:
        slope: 99.93348
        offset: 92.565
      torque_can_id: 0x18FA8032
      sensor_base_can_id: 0x18FA8100
      sensor_count: 13
```

## Testen ohne Hardware

Da kein robotkernel-Runtime und keine Sensor-Hardware vorhanden sind, kann das Modul nicht end-to-end ausgefuehrt werden. Es gibt aber mehrere Moeglichkeiten, die Funktionalitaet zu verifizieren.

### 1. Strukturelle Code-Pruefung

Checkliste fuer Code-Review:

- [ ] `MODULE_DEF(module_melectric_torque, melectric::torque_sensor)` vorhanden
- [ ] `static_assert(sizeof(torque_sensor_output_pd) == 103)` kompiliert
- [ ] Alle 6 State-Transitions implementiert (init↔preop↔safeop↔op)
- [ ] `tick()` ohne Heap-Allokation oder Blocking-Calls
- [ ] CAN-Filter-Masken: `CAN_EFF_MASK | CAN_EFF_FLAG` (Torque), `0x1FFFFF00 | CAN_EFF_FLAG` (Sensoren)
- [ ] Byte-Order: Big-Endian (`data[1] << 8 | data[2]`)
- [ ] Kalibrierformel: `(raw - 92.565) / 99.93348`
- [ ] `can_socket_` initialisiert auf `-1`, Cleanup im Destruktor
- [ ] `pd_data_` mit `memset` zero-initialisiert
- [ ] Stale-Counter initialisiert auf `UINT32_MAX` (Start: stale)
- [ ] `atomic<bool> tare_pending_` mit `exchange(false)` Pattern

### 2. vCAN-Simulation (ohne Sensor, ohne robotkernel)

Das CAN-Frame-Parsing kann isoliert getestet werden, indem man ein vCAN-Interface erstellt und Frames manuell injiziert. Dies testet den SocketCAN-Teil des Codes.

**vCAN einrichten:**

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

**Test-Frames senden** (in einem separaten Terminal):

```bash
# Torque-Frame: 50.00 Nm → raw = (50.0 * 99.93348 + 92.565) = 5089
# 5089 = 0x13E1 → Bytes: 08 13 E1 00 00 00 00 E0
cansend vcan0 18FA8032#0813E10000000000

# Sensor 0: X=100, Y=-200, Z=300
# 100=0x0064, -200=0xFF38, 300=0x012C
cansend vcan0 18FA8100#0064FF38012C

# Sensor 5: X=0, Y=0, Z=0
cansend vcan0 18FA8105#000000000000

# Tare-Command (normalerweise Output, aber zum Testen):
cansend vcan0 18FA8032#8900000000000000
```

**Mitlesen** (verifiziert, dass Frames auf dem vCAN ankommen):

```bash
candump vcan0
```

**Kontinuierlich Frames generieren** (simuliert Sensor-Betrieb):

```bash
# Torque alle 2 ms (500 Hz)
while true; do cansend vcan0 18FA8032#0813E10000000000; sleep 0.002; done

# Sensor 0 alle 10 ms (100 Hz)
while true; do cansend vcan0 18FA8100#0064FF38012C; sleep 0.01; done
```

### 3. Standalone-Testprogramm (SocketCAN-Parsing)

Um das CAN-Parsing isoliert zu testen, kann ein minimales C-Programm die gleiche Socket-Logik verwenden:

```c
// test_can_read.c - Kompiliert OHNE robotkernel-Abhaengigkeit
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>

int main() {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    printf("Listening on vcan0...\n");
    struct can_frame frame;
    while (read(sock, &frame, sizeof(frame)) > 0) {
        uint32_t id = frame.can_id & 0x1FFFFFFF;

        if (id == 0x18FA8032 && frame.can_dlc == 8) {
            int16_t raw = (int16_t)((frame.data[1] << 8) | frame.data[2]);
            double nm = ((double)raw - 92.565) / 99.93348;
            printf("TORQUE: raw=%d  %.3f Nm\n", raw, nm);
        }
        else if (id >= 0x18FA8100 && id <= 0x18FA810C && frame.can_dlc == 6) {
            int sensor = id - 0x18FA8100;
            int16_t x = (int16_t)((frame.data[0] << 8) | frame.data[1]);
            int16_t y = (int16_t)((frame.data[2] << 8) | frame.data[3]);
            int16_t z = (int16_t)((frame.data[4] << 8) | frame.data[5]);
            printf("SENSOR[%d]: X=%d Y=%d Z=%d\n", sensor, x, y, z);
        }
    }
    close(sock);
    return 0;
}
```

```bash
gcc -o test_can_read test_can_read.c
./test_can_read
# In einem anderen Terminal: cansend vcan0 18FA8032#0813E10000000000
# Erwartete Ausgabe: TORQUE: raw=5089  49.979 Nm
```

### 4. can-utils installieren

```bash
sudo apt install can-utils    # candump, cansend, cangen, cansniffer
```

## Process Data Layout (103 Bytes)

| Offset | Typ | Feld | Beschreibung |
|--------|-----|------|-------------|
| 0 | int16 | `torque_raw` | Roh-CAN-Wert (Torque * 100) |
| 2 | double | `torque_nm` | Kalibrierter Wert in Nm |
| 10 | int16[3] x13 | `sensor[0..12].x/y/z` | 13 Magnetfeldsensoren |
| 88 | uint8 | `torque_valid` | 1 = aktuell, 0 = stale |
| 89 | uint16 | `sensors_valid_mask` | Bit 0-12: je 1 Bit pro Sensor |
| 91 | uint32 | `torque_frame_count` | Empfangene Torque-Frames |
| 95 | uint32 | `sensor_frame_count` | Empfangene Sensor-Frames |
| 99 | uint32 | `error_count` | CAN-Lesefehler |

## Kalibrierung

Aus dem Kalibrierprotokoll (S001, 19.11.2025):

```
torque_nm = (raw_can_value - 92.565) / 99.93348
```

- Messbereich: +/- 100 Nm
- Linearitaetsabweichung inkl. Hysterese: +0,51 % / -0,37 %

## Lizenz

LGPL-3.0 (siehe [LICENSE](LICENSE))
