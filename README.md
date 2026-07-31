# Sim Racing Control Box

Firmware for a DIY sim racing control box based on an ATmega32U4 board such as Arduino Micro or Pro Micro.

The sketch exposes a stable USB HID joystick with 32 buttons and one analog handbrake axis. Profiles remap the physical controls for different sims without changing the USB device identity.

## Hardware target

- Board: Arduino Micro / Pro Micro / ATmega32U4
- FQBN: `arduino:avr:micro`
- HID library: Matthew Heironimus Arduino Joystick Library
- Matrix: 4 rows x 5 columns, diode protected
- Rotary encoders: two direct CLK/DT pairs; encoder push switches are part of the matrix
- Analog handbrake: `A3`
- RGB profile LED: `A0`, `A1`, `A2`
- Optional single LED: pin `9`

## Pin map

| Function | Pins |
| --- | --- |
| Matrix rows | `3`, `2`, `1`, `0` |
| Matrix columns | `4`, `5`, `6`, `7`, `8` |
| Encoder 1 CLK/DT | `15`, `14` |
| Encoder 2 CLK/DT | `16`, `10` |
| Handbrake analog input | `A3` |
| RGB LED R/G/B | `A0`, `A1`, `A2` |
| Single LED 1 | `9` |

## Profiles

The firmware currently includes profiles for:

- Generic
- Assetto Corsa
- Assetto Corsa Competizione
- WRC10
- Automobilista

Hold both encoder push buttons for two seconds to cycle the active profile. The selected profile is saved to EEPROM.

## Build locally

Install Arduino CLI, the AVR core, and the Joystick library, then compile:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install --git-url https://github.com/MHeironimus/ArduinoJoystickLibrary.git
arduino-cli compile --fqbn arduino:avr:micro .
```

## Upload

Connect the Arduino Micro / Pro Micro and check its port:

```bash
arduino-cli board list
```

Then upload, replacing the port if needed:

```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:micro .
```

ATmega32U4 boards can briefly disconnect and reconnect during bootloader reset. If upload timing fails, start the upload and press reset when the CLI begins looking for the port.

## Current verification

Compiled successfully on this machine with:

```bash
arduino-cli compile --fqbn arduino:avr:micro /home/joaquin-gomez/Arduino/SimRacingControlBox
```

Result:

```text
Sketch uses 10936 bytes (38%) of program storage space. Maximum is 28672 bytes.
Global variables use 1079 bytes (42%) of dynamic memory, leaving 1481 bytes for local variables. Maximum is 2560 bytes.
```
