# Aufgabenstellung: module_melectric_torque

## Kontext

Am DLR (Deutsches Zentrum fuer Luft- und Raumfahrt) wird ein **MELECTRIC Drehmomentsensor-Prototyp** (Seriennummer S001) in ein bestehendes Robotik-Framework integriert. Das Framework heisst **robotkernel-hal** (Release 6.0.0, LGPL-3.0) und folgt einer Mikrokernel-Architektur, in der Hardware-Treiber als dynamisch ladbare Module (`.so`-Plugins) implementiert werden.

## Ziel

Entwicklung eines neuen robotkernel-hal-Moduls **`module_melectric_torque`**, das:

1. **CAN-Frames** des Drehmomentsensors ueber ein Linux-vCAN-Interface empfaengt
2. Das **Sensor-Protokoll** dekodiert (Drehmoment + 13 Magnetfeld-Sensoren)
3. Die Daten als **Process Data** im robotkernel-Shared-Memory-Bus bereitstellt
4. Einen **Tare-Befehl** (Nullpunkt-Kalibrierung) an den Sensor senden kann

## Hardware-Signalkette

```
Drehmomentsensor (CAN-Bus)
    → Beckhoff EK1100 (EtherCAT Bus Coupler)
        → Beckhoff EL6751 (EtherCAT-to-CAN Gateway)
            → Linux vCAN Interface (vcan0)
                → DIESES MODUL (CAN-Frame-Parsing, Datenextraktion)
```

Der EL6751 wird durch das bestehende `module_el6751` angesteuert und stellt optional ein vCAN-Interface bereit, ueber das Standard-Linux-Tools und unser Modul direkt CAN-Frames lesen und schreiben koennen.

## Sensor-Protokoll

Quelle: `prototype_doc.pdf` (Sensor-Datenblatt des Herstellers)

### Ausgehende CAN-Nachrichten (Sensor → Modul)

| Nachricht | CAN ID (extended) | DLC | Rate | Inhalt |
|---|---|---|---|---|
| Drehmoment | `0x18FA8032` | 8 | 500 Hz | B0=`0x08`, B1-B2=Torque*100 (int16, signed), B3-B6=`0x00`, B7=`0xE0` |
| Sensor 0-12 | `0x18FA8100` bis `0x18FA810C` | 6 | 100 Hz | X, Y, Z (je int16, signed) |

- Sensoren 0-7: Drehmomentsignal-Komponenten (Magnetfeld)
- Sensoren 8-12: Externe Feldkompensation

### Eingehende CAN-Nachricht (Modul → Sensor)

| Nachricht | CAN ID (extended) | DLC | Inhalt |
|---|---|---|---|
| Tare (Nullen) | `0x18FA8032` | 8 | B0=`0x89`, B1-B7=`0x00` |

### Kalibrierung

Aus dem Kalibrierprotokoll (S001, 19.11.2025):

| Parameter | Wert |
|---|---|
| Steigung (slope) | 99,93348 Nm/Nm |
| Offset | 92,565 Nm |
| Linearitaetsabweichung inkl. Hysterese | +0,51 % / -0,37 % |
| Messbereich | +/- 100 Nm |

Umrechnung Rohwert zu Nm: `torque_nm = (raw_can_value - 92.565) / 99.93348`

## Architektur-Entscheidung: Direkter SocketCAN-Zugriff

Das Modul liest **direkt** vom Linux-vCAN-Device ueber die Standard-SocketCAN-API. Es nutzt NICHT den internen Stream-Mechanismus des EL6751-Moduls. Die Entkopplung ist bewusst:

- **Keine Code-Level-Abhaengigkeit** auf `module_el6751`
- **Nur Deployment-Abhaengigkeit**: el6751 muss vorher laufen und vcan0 beschreiben
- Standard-Linux-Debugging-Tools (`candump`, `cansend`, `cangen`) funktionieren direkt

### Voraussetzung: vCAN-Device

Das vCAN-Interface muss VOR dem Start des Moduls existieren (wird nicht automatisch erstellt):

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Anforderungen an das Modul

### Funktionale Anforderungen

1. **CAN-Frame-Empfang**: Non-blocking Read aller verfuegbaren Frames pro Tick-Zyklus (1 kHz)
2. **Torque-Dekodierung**: int16 aus Bytes 1-2 extrahieren, mit Kalibrierung in Nm umrechnen
3. **Sensor-Dekodierung**: 13 Sensoren mit je X/Y/Z (int16) aus separaten CAN-Frames
4. **Process Data**: Alle Werte als flachen Byte-Buffer (103 Bytes) im Shared-Memory-Bus exponieren
5. **Stale-Data-Erkennung**: `torque_valid`-Flag und `sensors_valid_mask` pro Tick aktualisieren
6. **Tare-Command**: Moeglichkeit, den Nullpunkt-Kalibrierbefehl an den Sensor zu senden
7. **Frame-Zaehler**: Separate Zaehler fuer Torque-Frames, Sensor-Frames und Fehler

### Nicht-funktionale Anforderungen

1. **Echtzeit-faehig**: `tick()` muss deterministisch sein - keine Heap-Allokation, kein Blocking
2. **Framework-konform**: Korrekte Implementierung der State Machine (INIT → PREOP → SAFEOP → OP)
3. **Build-System**: GNU Autotools (configure.ac.in, Makefile.am, bootstrap.sh) nach Framework-Standard
4. **Lizenz**: LGPL-3.0 (wie das gesamte Framework)

### Process Data Struktur (Output)

```
torque_raw          (int16)    - Roh-CAN-Wert (Torque * 100)
torque_nm           (double)   - Kalibrierter Wert in Nm
sensor[0..12].x/y/z (int16)   - 13 Sensoren, je 3 Achsen
torque_valid        (uint8)    - 1 = aktuell, 0 = stale
sensors_valid_mask  (uint16)   - Bit 0-12: je 1 Bit pro Sensor
torque_frame_count  (uint32)   - Empfangene Torque-Frames
sensor_frame_count  (uint32)   - Empfangene Sensor-Frames
error_count         (uint32)   - CAN-Lesefehler
```

Gesamtgroesse: 103 Bytes. Device-Name: `<module_name>.data.outputs.pd`

## Lieferumfang (zu erstellende Dateien)

```
module_melectric_torque/
  src/
    melectric_torque.h         # Klassen-Header
    melectric_torque.cpp       # Implementierung
    Makefile.am                # Build-Regeln fuer src/
  Makefile.am                  # Root-Makefile
  configure.ac.in              # Autoconf-Template
  bootstrap.sh                 # Build-Initialisierung
  module_melectric_torque.pc.in  # pkg-config
  LICENSE                      # LGPL-3.0
```

## YAML-Konfiguration des Moduls

```yaml
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

## Design-Entscheidungen

Ergebnis des Brainstormings vom 12.02.2026:

| Entscheidung | Ergebnis | Begruendung |
|---|---|---|
| **Scope V1** | Komplett in einem Zug | Spec ist detailliert genug, kein inkrementeller Ansatz noetig |
| **Tare-Command** | robotkernel-Service | Sauberer als PD-Flag, ermoeglicht Ausloesung ueber REST/CLI-Bridges |
| **Testing** | Unit-Tests mit gemockten CAN-Frames | Portabel, kein vCAN-Device oder Hardware noetig |
| **Error-Handling** | ERROR-State bei Socket-Fehler | Standard-Pattern der Framework-State-Machine, klare Signalisierung |
| **Sensor-Kalibrierung** | Nur Rohdaten (int16) | YAGNI, keine Kalibriertabellen vom Hersteller vorhanden |
| **SSI-Interface** | Nicht implementieren | Nur CAN wird genutzt, SSI ist nicht benoetigt |

### Details zu den Entscheidungen

**Tare als Service**: Das Modul registriert einen robotkernel-Service, der von externen Bridges (REST, CLI, Links-and-Nodes) aufgerufen werden kann. Der Service sendet den Tare-Frame (`0x89`) ueber den CAN-Socket. Kein Input-Process-Data noetig.

**Unit-Tests**: CAN-Frame-Parsing wird mit hart codierten Byte-Arrays getestet. SocketCAN-Syscalls werden gemockt. Testfaelle:
- Torque-Frame Dekodierung (positiv, negativ, Grenzwerte)
- Sensor-Frame Dekodierung (alle 13 Sensoren)
- Kalibrierung Rohwert → Nm
- Ungueltige Frames (falsches DLC, unbekannte CAN-ID)
- Stale-Data-Erkennung

**ERROR-State**: Wenn `socket()`, `ioctl()` oder `bind()` in `set_state_init_2_preop()` fehlschlagen (z. B. vcan0 existiert nicht), geht das Modul in den ERROR-State. Das Framework entscheidet ueber Retry oder Abbruch.

## Referenzen

- Sensor-Datenblatt: `/mnt/c/temp/prototype_doc.pdf`
- Upstream module_el6751: https://github.com/robotkernel-hal/module_el6751 (Branch: release/6.0.0)
- Upstream module_tty (Referenz-Pattern): https://github.com/robotkernel-hal/module_tty (Branch: release/6.0.0)
- Upstream robotkernel Core API: https://github.com/robotkernel-hal/robotkernel (Branch: release/6.0.0)
