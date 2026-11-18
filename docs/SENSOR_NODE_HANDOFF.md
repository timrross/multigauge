# Sensor Node Handoff (VW T4 digital dash)

Written 2026-11-10 from a long design conversation on claude.ai. Revised the same day (revision 2) by a local Claude Code session that read the firmware repo and reviewed the board plan. Section 2a lists what revision 2 changed. Section 6 describes the firmware as it actually is in the repo.

## 1. Project in one paragraph

Tim is building a digital dash for a VW T4 (1.9 TD ABL, mechanical injection, no engine ECU). Two nodes talk over CAN:

- **Display node** in the dash: Adafruit Qualia ESP32-S3 driving a round TFT in a printed ASA-CF housing. Built and working, waiting for CAN data. Its Waveshare SN65HVD230 CAN module is already mounted in the housing.
- **Sensor node** in the engine bay, in its own enclosure near the battery box (away from the heat and dirt of the engine): Waveshare ESP32-C3-Zero reading oil pressure, boost pressure, oil temp, intake temp and EGT, then sending them over CAN. Only the CAN pair goes through the bulkhead, which is the reason for having two nodes.

The firmware started as one monolithic codebase on the S3. Claude Code already refactored it into two MCUs plus CAN. That refactor has not been bench tested, and it predates the hardware changes in section 2.

## 2. What changed since the firmware refactor

1. **Hella 6PR 010 378-207 combined oil sensor is removed.** Its PWM decoding (RMT or interrupt timing, 9216 us frame) is no longer needed. It is replaced by two plain analogue sensors.
2. **All analogue sensors now go through an ADS1115** over I2C. The C3's internal ADC is not used.
3. **GPIO assignments changed** (section 4).
4. **A custom through-hole carrier PCB** replaces the old perf board. All four modules plug into sockets on it.
5. **The TPS61023 5V boost module is no longer needed.** The sensors get 5V from the buck converter.

## 2a. Revision 2 changes (board review, 2026-11-10)

1. **R7 and R8 added.** A 10k series resistor sits between each NTC node and its ADC pin, and the 100nF filter caps C5 and C6 moved to the ADC side. A loom wire shorted to 12V now pushes about 1 mA into the ADC instead of killing it, and the filter cutoff drops from several kHz to about 150 Hz. Cost: under 0.2C of error.
2. **D1 added.** A 1N5817 Schottky feeds the C3's 5V pin from the +5V rail. USB and the buck can now be connected at the same time without the two supplies fighting.
3. **The boost sensor has its own 5V and GND on J1**, so each pressure sensor has its own supply pair and the loom needs no splice. J1 is grouped by sensor (reordered 2026-11-10, see the J1 table in section 7). Only pins 11 and 12 remain spare, so J3 shrinks to 2 pins.
4. **J4 added.** A 4-pin I2C expansion header (3V3, GND, SDA, SCL) for a BME280 or a second ADS1115 later.
5. **TP1 to TP7 added.** Test points on +5V, +3V3, GND and the four ADC inputs.
6. **R1 to R8 are 1% metal film.** With 5% parts the divider ratio could be 4% out and oil temp about 2C out at 100C.
7. **L1 needs a DC resistance under 0.5 ohm.** The sensors are fed through it and 5V is the oil pressure sensor's minimum supply.
8. **Modules get soldered in after bench testing** (or use turned-pin sockets). Plain sockets work loose with diesel vibration.
9. **Enclosure location decided:** near the battery box.

## 3. Hardware on the sensor node

| Part | Detail |
| --- | --- |
| MCU | Waveshare ESP32-C3-Zero (ESP32-C3, 4MB flash, USB-Serial/JTAG). Two 1x9 pin rows, 15.24mm apart |
| ADC | ADS1115 breakout, 10 pins in one row: VDD, GND, SCL, SDA, ADDR, ALRT, A0, A1, A2, A3. ADDR tied to GND, so I2C address 0x48. Has its own I2C pull-ups |
| Thermocouple amp | MAX31855 breakout, 6 pins: Vin, 3Vo, GND, DO, CS, CLK. Powered from 5V on Vin. K-type EGT probe wired straight to its screw terminal |
| CAN transceiver | Waveshare SN65HVD230 module, 4 pins: 3V3, GND, CAN RX, CAN TX. Bus wires go to its own screw terminal. Has a 120 ohm termination jumper |
| Oil pressure | 1/8 NPT transducer, 0 to 150 PSI, 0.5 to 4.5V output, 5 to 16V supply. Red = supply, black = GND, green = output |
| Boost pressure | Round 3-wire sensor (red, black, green). Tim reports 0 to 5V output. No part number, range unconfirmed. An existing calibration equation is in the old code |
| Oil temp | 1/8 NPT two-wire NTC sender, 2031 ohm at 20C, 108 ohm at 100C |
| Intake temp | Two-wire NTC in the side of the intercooler, about 2.1k at 25C. Not shared with anything |
| EGT | K-type thermocouple |

Power: 12V, 1A blade fuse, 1.5KE36A TVS plus 100uF/50V plus 100nF across the buck input, GYVRM potted buck (8 to 40V in, 5V 5A out), then on the carrier board a 47uH series inductor (L1) followed by 100uF/25V plus 100nF. That filtered +5V rail feeds the pressure sensors and the MAX31855 directly, and the C3 through Schottky diode D1. The display node has the same chain with a 2A fuse.

Power notes from the review:

- L1 must be a low resistance part (under 0.5 ohm, rated 500 mA or more). Measure it with a multimeter. Small axial 47uH parts are often 1 ohm or more, which would drop the sensor supply below 5V.
- Measure the +5V rail at J1 pin 1 once the board is built and keep it as a firmware constant. If the pressure sensors are ratiometric (output scales with supply), any error on this rail is a direct pressure error.
- The 1.5KE36A clamps at about 50V at full surge current, above the buck's 40V limit. A 1.5KE27A (clamps about 37V, stands off 23V) is a better fit. Low priority, off board.
- Use 105C rated electrolytics and ferrules on stranded wire in the screw terminals.

## 4. Pin map (firmware must match this)

| GPIO | Function | Goes to |
| --- | --- | --- |
| GP0 | CAN TX (TWAI) | CAN module "CAN TX" pin |
| GP1 | CAN RX (TWAI) | CAN module "CAN RX" pin |
| GP3 | SPI MISO | MAX31855 DO |
| GP4 | I2C SDA | ADS1115 SDA |
| GP5 | I2C SCL | ADS1115 SCL |
| GP6 | SPI SCK | MAX31855 CLK |
| GP7 | SPI CS | MAX31855 CS |

Left free on purpose: GP2, GP8 and GP9 (boot strapping pins), GP10 (onboard RGB LED), GP18 and GP19 (USB), GP20 and GP21.

Module silkscreen "CAN TX" is the transceiver's data input, so it connects to the MCU's TWAI TX. If the bus stays silent on the bench, swapping GP0 and GP1 in firmware is the first thing to try.

## 5. ADS1115 channels and conversions

Use the +/-4.096V gain setting (0.125 mV per bit), single-ended reads. All four inputs stay below 3.3V. Each input has a 100nF capacitor to ground right at the ADC pin.

| Channel | Signal | Front end | Voltage at the ADC |
| --- | --- | --- | --- |
| A0 | Oil pressure | 10k over 15k divider (R1, R2), ratio 0.6 | 0.30V (0 PSI) to 2.70V (150 PSI) |
| A1 | Boost pressure | 10k over 15k divider (R3, R4), ratio 0.6 | 0V to 3.0V |
| A2 | Oil temp NTC | 330 ohm pull-up to 3V3 (R5), NTC to GND, then 10k in series (R7) | 2.84V at 20C, 0.81V at 100C, 0.43V at 130C |
| A3 | Intake temp NTC | 2.2k pull-up to 3V3 (R6), NTC to GND, then 10k in series (R8) | 1.77V at 20C, 0.72V at 60C, 0.26V at 100C |

The 10k series resistors work against the ADS1115's input resistance (about 6 megohm at this gain), so the NTC channels read about 0.17% low. That is under 0.2C and can be ignored or folded into the pull-up constant.

Conversions:

```
Oil pressure:   V_sensor = V_adc / 0.6
                PSI = (V_sensor - 0.5) * 37.5        (bar = PSI * 0.0689476)

Boost pressure: V_sensor = V_adc / 0.6
                PSI = 12.864 * V_sensor - 12.877
                (the existing calibration. It works on the sensor's own
                voltage, so only the divider ratio changes, not the equation.)

NTC channels:   R = R_pullup * V_adc / (V_3v3 - V_adc)
                1/T_kelvin = A + B*ln(R) + C*ln(R)^3
                T_celsius = T_kelvin - 273.15
```

Steinhart-Hart coefficients:

| | Oil temp (A2) | Intake temp (A3) |
| --- | --- | --- |
| R_pullup | 330 | 2200 |
| A | 1.511193e-3 | 1.357216e-3 |
| B | 2.489549e-4 | 2.446728e-4 |
| C | 1.298416e-8 | 2.843971e-7 |
| Source | Least squares fit to the manufacturer's 16-point table, 20 to 170C. Fits within 0.5C. Sensor tolerance is +/-5% | Tim's own three-point fit (3.1C 5050 ohm, 50C 849 ohm, 71.4C 427 ohm). Within 0.8C of his four other measurements |

Oil sender table used for the fit (C, ohm): 20 2031, 30 1286, 40 843.9, 50 569.9, 60 388, 70 277.8, 80 200, 90 146.7, 100 108, 110 82.7, 120 63.5, 130 49.3, 140 38.9, 150 30.4, 160 24.4, 170 19.8.

Make these compile-time constants in one place: divider ratio per channel, pull-up values, V_3v3 and V_5V (measure the real rails), and the coefficients. With 1% resistors the divider ratios should be within 0.8%, so trimming against a multimeter is optional.

Fault handling to add:

- NTC channel above about 3.2V = open circuit, below about 0.1V = short.
- Oil pressure sensor output below about 0.25V or above about 4.75V (before the divider) = sensor or wiring fault.
- MAX31855 fault bits (open, short to GND, short to VCC).
- A per-value "invalid" state over CAN is the natural fit. Note the display only shows "--" at boot. After that it ignores invalid values and keeps the last good reading on screen, so the display side needs work too (section 6).

## 6. Firmware state

Updated 2026-11-10. The firmware now matches the carrier board and the bugs from the review are fixed. The changes are uncommitted in the working tree. All four PlatformIO targets build, and the conversion maths and CAN encode/decode pass a PC-side test against the numbers in section 5. **Nothing has been run on hardware yet.** Flash both nodes together: the oil frame's status bits changed, so an old node and a new node disagree.

Where things live on the sensor node:

| File | Contents |
| --- | --- |
| `src/sensor_mcu/sensor_config.h` | Every board-specific number: pins, ADC channels, divider ratios, pull-ups, rail voltages, coefficients, fault limits, filter speeds |
| `src/sensor_mcu/conversions.h` | Pure maths: ADC volts to value plus fault code. No hardware access, so it can be tested on a PC |
| `src/sensor_mcu/sensor.cpp` | ADS1115 and MAX31855 access |
| `src/sensor_mcu/can_tx.cpp` | CAN send and bus recovery |
| `include/shared/can_protocol.h` | Frame layouts, shared by both nodes |

What changed on the sensor node:

- Pins, ADC channel order, divider ratios and pull-ups match sections 4 and 5. Oil temp uses the section 5 coefficients.
- Oil pressure: fault limits are 0.25V and 4.75V at the sensor. Between 0.25V and 0.5V it reads 0 bar instead of going invalid.
- Boost: unplugged (below 0.2V) or shorted high (above 4.9V) is now a fault. These two limits are a guess until the sensor is checked on the bench.
- NTC channels: below 0.1V is a short. Open circuit is a reading within 1.5% of the 3V3 rail for oil temp (3% for intake). A very cold oil sender also reads close to the rail, so oil temp shows "--" below about -24C, and the limit only works if `V_3V3_RAIL` is set from a real measurement.
- EGT: the MAX31855 fault bits are reported, and an all-zero reply (chip absent or unpowered) is a fault. Readings from -50C to 1300C are accepted.
- Smoothing is per sensor and resets after a fault. Time to show 90% of a change: boost 0.45 s, oil pressure 1.7 s, EGT 2.3 s, oil temp 4 s, intake 6.5 s.
- ADS1115 reads have a 30 ms time limit, and every I2C transaction is checked. A missing or dead ADS1115 no longer stops the node: it keeps sending heartbeats, reports the fault and keeps looking for the module.
- A 5 second watchdog covers all three tasks. Start-up failures restart the chip instead of hanging.
- CAN bus-off recovery now restarts the driver.
- `V_3V3_RAIL` and `V_5V_RAIL` are constants to set from multimeter readings on the real board.

What changed in the CAN protocol (IDs, bitrate and frame sizes are unchanged):

- Oil frame (0x101): temperature and pressure each have their own valid bit, so a fault on one no longer blanks the other.
- Every frame carries a fault code for diagnostics.
- Heartbeat (0x1FF) carries a bit per faulted sensor.

What changed on the display node:

- An invalid value shows "--" instead of holding the last good number. The boost needle drops to zero.
- A value that stops arriving is blanked after 1 s (boost) or 3 s (the rest). Everything blanks if the heartbeat is lost for 3 s.
- Readouts turn red past a warning threshold: oil pressure below 0.8 bar, oil temp above 130C, EGT above 700C. These are starting values in `src/display_mcu/ui.cpp` and need tuning to the engine. Oil pressure is also red with the engine off, like an oil light.
- Short, remote and extended frames are ignored. Sensor node reboots and missed heartbeats are logged in debug builds.
- The connection dot is only redrawn when it changes. One LVGL call at boot now takes the GUI mutex.

Not done, on purpose:

- No watchdog on the display node. A false trigger there would reboot the screen while driving, and its timing under LVGL has not been measured.
- The heartbeat fault bits are received and logged but not shown on screen.
- The BMP280 is still unimplemented on both nodes (library in `lib_deps`, `ENABLE_ATMOS_SENSOR` 0).
- The display's TWAI pins (GP16 TX, GP17 RX) still need checking against the real wiring.

Bench test order: ADS1115 alone with a potentiometer on each input (use the test points), then real sensors at room temperature against the voltages in section 5, then CAN between the two nodes with both 120 ohm terminations fitted. With power off and both nodes connected, a multimeter across CANH and CANL should read about 60 ohm.

Flashing: esptool is installed through pipx on Tim's Mac. PlatformIO and the Arduino IDE are also installed. The C3 appears as /dev/cu.usbmodem*. Close the Arduino IDE serial monitor first or the port is busy. On the carrier board D1 makes it safe to have USB and the buck connected together. On the old perf board (no D1) never connect both.

## 7. Carrier board (KiCad, Tim's first PCB)

All through-hole, 2 layer, 85.09 x 66.04mm. The KiCad 10 project is in `hardware/sensor-node-carrier/`:

| File | What it is |
| --- | --- |
| `sensor-node-carrier.kicad_pro` | Open this in KiCad |
| `sensor-node-carrier.kicad_sch` | Schematic |
| `sensor-node-carrier.kicad_pcb` | Board: placed, routed, GND pour on both layers |
| `SensorNode.kicad_sym`, `SensorNode.pretty/` | Project symbols and footprints for the four modules |
| `out/schematic.pdf` | Schematic printout |
| `out/paper-test-fit-1to1.pdf` | Print at 100% for the paper test fit |
| `out/sensor-node-carrier-gerbers.zip` | Manufacturing files for the board house |
| `tools/` | The scripts that generated all of the above. `tools/build.sh` regenerates everything, including the PDFs and the zip, and overwrites hand edits |

The files were generated by script and then checked with kicad-cli, not by eye alone: ERC 0 violations, schematic netlist identical to the netlist table below, DRC 0 violations, 0 unconnected pads, 0 schematic parity issues. The routing came from a simple autorouter (0.5mm signal, 1.0mm power, 0.2mm clearance, 4 vias). It is electrically correct but not pretty, so reroute anything you like.

**Not verified, because it needs the real parts in hand.** Check all of these on the paper print before ordering:

1. U1 orientation. The footprint assumes that with the C3-Zero seen from above and USB at the top, the left row reads 5V, GND, 3V3, GP0 to GP5 and the right row reads GP21 (TX), GP20 (RX), GP19, GP18, GP10, GP9, GP8, GP7, GP6. If the real board is mirrored, the footprint is wrong.
2. Module outlines. U2 is an estimate (28 x 18mm) that Tim checked by laying the module on the print. U3 and U4 are as Tim measured them on 2026-11-10. U3 is 19.5mm across and 22.5mm long, with the Vin pin 3mm from the side and the pin row 2mm in from the end. U4 is 10 x 26mm (the 26mm includes the screw terminal). Both lie flat with their screw terminals facing the right-hand edge. The U3 outline ends 1.09mm inside the board edge and the U4 outline 0.90mm, which is level to within the 0.635mm placement grid. Two assumptions: the 22.5mm for U3 is taken as the overall length including its green terminal, and U4 is taken as 24.5mm from the centre of the pin holes to the terminal face. If either real figure is larger, that gap shrinks by the difference. The J4 labels are on the left of the header because the U3 pin names use the space on its right.
3. Pin 1 end of each single-row module. Every socket pin has its name on the silkscreen, so compare the print against each module's own labels.
4. Screw terminal wire entry. J1 faces the bottom edge and J2 faces the left edge, assuming the KiCad Phoenix MKDS footprint matches the terminals in the parts box.
5. Part sizes: L1 is drawn as a radial drum inductor, 8mm diameter with holes 5.08mm apart, for the 47uH "0810" part Tim ordered from AliExpress on 2026-11-11 (8 x 10mm body; the real part arrived 2026-11-17 and its legs read about 5mm apart against a ruler, body about 8mm), C1 as 6.3mm diameter on 2.5mm pitch (chosen to take the capacitors Tim has: a loose 100uF 50V that looks about 6.3 x 11mm, or the kit 100uF 25V at 5 x 11mm with 2mm legs), the 100nF caps on 5mm pitch (Tim measured 5mm legs on his 104 caps, 2026-11-11). The screw terminals are drawn at 5.0mm pin spacing, as Tim measured on 2026-11-11. They were 5.08mm before, which would have put the 12th pin 0.88mm out.
6. Checked from Tim's photos against a ruler, 2026-11-17: D1 (body about 4.9 x 2.3mm, fits the DO-41 footprint) and the resistors (1/4W 1% kit, 5 x 2mm bodies, fits 10.16mm pitch). Tim is fitting his loose 100uF 50V as C1 and his own 104 ceramics as C2 to C6. The 1.5KE27A TVS also arrived. It is off board.

**Everything regenerated 2026-11-18** with `tools/build.sh` (also run 2026-11-17): ERC 0, netlist matches, DRC 0, 0 unconnected, 0 parity issues. The board is unchanged since the 2026-11-11 commit. Every layer was compared pixel for pixel at 600 dpi against the previous build, and the drill files are identical. Only dates, internal IDs and the order of items in the files changed. `build.sh` now also writes `out/schematic.pdf` and `out/paper-test-fit-1to1.pdf`, so the print always matches the Gerbers. The outline on the print measures 85.1 x 66.1mm, so check that with a ruler after printing at 100%. Drill sizes: 0.4mm (vias), 0.8, 1.0, 1.1, 1.3mm plated, 3.2mm unplated.

**Checked against photos of Tim's modules, 2026-11-10** (pin order and pin direction only, not measured sizes):

- U1 C3-Zero: confirmed. The underside labels give 5V, GND, 3V3, 0 to 5 on the left and 21, 20, 19, 18, 10, 9, 8, 7, 6 on the right with USB at the top, seen from above. Straight pins, component side up.
- U2 ADS1115: confirmed. VDD, GND, SCL, SDA, ADDR, ALRT, A0 to A3, straight pins, component side up. It has its own 10k pull-ups on SCL and SDA.
- U3 MAX31855: pin order confirmed (Vin, 3Vo, GND, DO, CS, CLK) but the header is **right-angle**, so it cannot lie flat as drawn. See open items.
- U4 CAN module: confirmed from a second set of photos. Component side up with the pins on the left it reads 3.3V, GND, CAN RX, CAN TX from the top, with CANL and CANH on the screw terminal. It is a small 10 x 26mm board, not the 16 x 29.5mm first assumed, so the footprint outline was redrawn. Right-angle header.
- Tim laid all four modules on the 1:1 print and the pin rows and outlines looked right.

**Paper test fit done, 2026-11-18.** Tim placed every part on the 1:1 print (photo: all four modules, L1, C1, C2 to C6, D1, R1 to R8, J1 and J2) and reported that it all fits. J1 is four 3-way blocks side by side, so slide them together before soldering. J2 is one 2-way block. Seen in the photo: module pin names line up with the silkscreen on all four modules, the D1 band is at the K end, and L1 clears C1 (the drawn gap is 1.4mm). Tim confirmed the wire entry sides: J1 faces the bottom edge and J2 the left edge. L1 and C1 lean in the photo only because they are loose on the paper. J3, J4 and the socket strips were not on the print. They are standard 2.54mm parts.

Parts:

| Ref | Part | Footprint |
| --- | --- | --- |
| U1 | ESP32-C3-Zero | 2 x 1x9 socket rows, 15.24mm apart |
| U2 | ADS1115 module | 1x10 socket |
| U3 | MAX31855 module | 1x6 socket |
| U4 | SN65HVD230 CAN module | 1x4 socket |
| J1 | Loom terminal | 12-way 5.0mm screw terminal |
| J2 | 5V in | 2-way 5.0mm screw terminal |
| J3 | Spare loom pins | 1x2 pin header |
| J4 | I2C expansion | 1x4 pin header |
| L1 | 47uH, under 0.5 ohm, 500 mA or more | Radial drum, 8mm, 5.08mm pitch (5mm legs) |
| C1 | 100uF, 25V or more, 105C | Radial, 6.3mm, 2.5mm pitch |
| C2 to C6 | 100nF | Disc, 5mm pitch |
| D1 | 1N5817 Schottky | DO-41, 10.16mm pitch |
| R1, R3, R7, R8 | 10k 1% | Axial, 10.16mm pitch |
| R2, R4 | 15k 1% | Axial, 10.16mm pitch |
| R5 | 330 ohm 1% | Axial, 10.16mm pitch |
| R6 | 2.2k 1% | Axial, 10.16mm pitch |
| TP1 to TP7 | Test points: +5V, +3V3, GND, A0, A1, A2, A3 | 2mm pad, 1mm hole |
| H1 to H4 | M3 mounting holes | 3.2mm |

Socket pin order, pin 1 first:

- U1 pins 1 to 9 (left row): 5V, GND, 3V3, GP0, GP1, GP2, GP3, GP4, GP5
- U1 pins 10 to 18 (right row): GP21, GP20, GP19, GP18, GP10, GP9, GP8, GP7, GP6
- U2: VDD, GND, SCL, SDA, ADDR, ALRT, A0, A1, A2, A3
- U3: Vin, 3Vo, GND, DO, CS, CLK
- U4: 3V3, GND, CAN RX, CAN TX
- J4: 3V3, GND, SDA, SCL

Loom terminal J1 (the loom ends in a Deutsch DT 12-pin, DT06-12SA and DT04-12PA):

| Pin | Signal | Silkscreen |
| --- | --- | --- |
| 1 | 5V to the oil pressure sensor | 5V |
| 2 | GND to the oil pressure sensor | GND |
| 3 | Oil pressure signal | OILP |
| 4 | 5V to the boost sensor | 5V |
| 5 | GND to the boost sensor | GND |
| 6 | Boost pressure signal | BST |
| 7, 8 | Oil temp signal, return | OILT, GND |
| 9, 10 | Intake temp signal, return | IAT, GND |
| 11, 12 | Spares, routed to J3 | SP1, SP2 |

Pin 1 is the square pad at the left-hand end. The wires of each sensor sit next to each other.

Netlist:

| Net | Connects |
| --- | --- |
| 5V_IN | J2 pin 1, L1 end A |
| +5V | L1 end B, C1 +, C2, D1 anode, U3 Vin, J1 pins 1 and 4, TP1 |
| +5V_MCU | D1 cathode, U1 5V |
| +3V3 | U1 3V3, U2 VDD, U4 3V3, R5 top, R6 top, J4 pin 1, TP2 |
| GND | J2 pin 2, C1 -, C2, U1 GND, U2 GND, U2 ADDR, U3 GND, U4 GND, J1 pins 2, 5, 8 and 10, R2 bottom, R4 bottom, C3 to C6 bottom, J4 pin 2, TP3 |
| OIL_P_RAW | J1 pin 3, R1 top |
| OIL_P | R1 bottom, R2 top, C3 top, U2 A0, TP4 |
| BOOST_RAW | J1 pin 6, R3 top |
| BOOST | R3 bottom, R4 top, C4 top, U2 A1, TP5 |
| OIL_T_RAW | J1 pin 7, R5 bottom, R7 end A |
| OIL_T | R7 end B, C5 top, U2 A2, TP6 |
| IAT_RAW | J1 pin 9, R6 bottom, R8 end A |
| IAT | R8 end B, C6 top, U2 A3, TP7 |
| SDA | U1 GP4, U2 SDA, J4 pin 3 |
| SCL | U1 GP5, U2 SCL, J4 pin 4 |
| TC_CLK | U1 GP6, U3 CLK |
| TC_CS | U1 GP7, U3 CS |
| TC_DO | U1 GP3, U3 DO |
| CAN_TX | U1 GP0, U4 CAN TX |
| CAN_RX | U1 GP1, U4 CAN RX |
| SPARE1, SPARE2 | J1 pins 11 and 12, J3 pins 1 and 2 |

No-connect: U1 GP2, GP8, GP9, GP10, GP18, GP19, GP20, GP21, U2 ALRT, U3 3Vo.

Layout as built: J1 along the bottom edge. U1 top centre with USB at the top edge. Power entry (J2, L1, C1, C2, D1) top left, with J2 on the left edge. U2 on the left with its analogue pins next to the filter caps. Dividers, pull-ups and test points in the middle between J1 and U2. U4 and U3 on the right with their screw terminals at the right edge. Nothing sits under a module.

Board house settings: 2 layer, 1.6mm, HASL. Smallest track 0.5mm, smallest gap 0.2mm, vias 0.8mm with a 0.4mm hole, which any board house can make. The manufacturing files are in `hardware/sensor-node-carrier/out/sensor-node-carrier-gerbers.zip` (7 Gerber layers plus 2 drill files). Upload that zip as it is, but only after the paper test fit. If the fit changes the board, the zip has to be exported again (`tools/build.sh` does it).

CAN bus between nodes: twisted pair (blue and white) straight into each CAN module's screw terminal, same colour on the same terminal at both ends, 120 ohm termination at both ends. Both nodes share the van's ground through their buck converters.

## 7a. Rev 3: SMD version for JLCPCB assembly (optional follow-on)

**Status 2026-11-11: shelved.** JLCPCB's quote for assembled boards came to over £100, so Tim is building rev 2. The files are kept in case several boards are ever wanted.

Started 2026-11-11 because Tim asked how small the board could go. It lives in `hardware/sensor-node-smd/` and does not touch rev 2. **Build and prove rev 2 first.** Rev 3 has passed KiCad's checks but nothing about it has been built or powered.

Size: 46 x 48mm, about 39% of the rev 2 area (85 x 66mm). The width is set by the loom connector plus two M3 holes. Another 4 to 5mm could come off the height with a re-layout.

What is different from rev 2:

- **Chips go straight on the board, no modules.** ESP32-C3-WROOM-02U (the module without the PCB antenna, since the radio is not used), ADS1115, MAX31855K, SN65HVD230, and an AP2112K 3.3V regulator.
- **Same circuit, same GPIO numbers.** The firmware and `sensor_config.h` need no changes. The PlatformIO board stays `lolin_c3_mini`.
- **One plug for the loom, power and CAN.** J1 is a Molex Micro-Fit 3.0 right-angle header, 2 x 8 (part 43045-1600). The mating plug is housing 43025-1600 with 43030 crimp terminals, which need a crimp tool or pre-crimped leads.
- **The thermocouple keeps its own 2-way screw terminal (J2)**, right next to the MAX31855.
- **USB-C socket, BOOT and RESET buttons, power LED** are on the board, because the C3-Zero module that provided them is gone. The LED runs from the regulator input (VSYS), so it lights on buck or USB power.
- **D2 added.** D1 (buck) and D2 (USB) both feed the regulator, so either supply can power the logic. The sensors only get 5V from the buck.
- **The spare loom pins and the I2C header are gone.** J1 pins 11 and 13 are unconnected spares. SDA and SCL have test pads.
- **CAN termination is on the board** (R17, 120 ohm). Cut the JP1 solder bridge to remove it.

J1 pin map (pin n and pin n+8 share a column in the plug):

| Front row | Function | Back row | Function |
| --- | --- | --- | --- |
| 1 | Intake temp NTC | 9 | Intake temp NTC return (GND) |
| 2 | Oil temp NTC | 10 | Oil temp NTC return (GND) |
| 3 | Boost signal | 11 | Spare, not connected |
| 4 | +5V to boost sensor | 12 | GND boost sensor |
| 5 | Oil pressure signal | 13 | Spare, not connected |
| 6 | +5V to oil pressure sensor | 14 | GND oil pressure sensor |
| 7 | CAN high | 15 | CAN low |
| 8 | +5V in from the buck | 16 | GND in |

The same table is printed on the underside of the board. J2: T+ and T- are printed next to the terminal. T- is the one nearer J1.

Files:

- `tools/design.py` is the single source of truth (parts, nets, J1 map, JLCPCB part numbers). `tools/build.sh` regenerates everything and re-runs the checks. It overwrites hand edits, and takes about three minutes because the router is slow.
- `out/sensor-node-smd-gerbers.zip`, `out/jlcpcb-bom.csv`, `out/jlcpcb-cpl.csv` are the three files JLCPCB asks for.
- `out/schematic.pdf`, `out/board-top.pdf`, `out/board-bottom.pdf` for reading.

Checks passed on 2026-11-11: ERC 0 violations, netlist identical to `design.py`, DRC 0 violations, 0 unconnected pads, 0 schematic parity issues. Design rules: 0.2mm track, 0.15mm gap, 0.6mm via with 0.3mm hole, all inside JLCPCB's standard 2-layer limits. All SMD parts are on the top side.

Ordering at JLCPCB: 2 layers, 1.6mm, 1 oz, PCB assembly on the top side. J1 and J2 are not in the BOM. Tim solders those two by hand.

Things to check before ordering rev 3:

1. **Part rotation in JLCPCB's placement preview.** KiCad and JLCPCB do not always agree on which way a part faces. Check pin 1 on U1 to U5, the stripe on D1 and D2, the LED D3, the + on C1, and the USB socket J3 against `out/board-top.pdf`, and rotate in their preview if needed.
2. **Stock and part numbers.** Every number in `design.py` was checked against JLCPCB's live catalogue on 2026-11-11 and all were in stock. Stock changes, so confirm each line when the BOM is uploaded. The resistors, small capacitors and Schottky diodes are JLCPCB "basic" parts. The other ten part types are "extended", which JLCPCB charges a one-off loading fee for.
3. **J2 footprint.** Drawn for a Phoenix MKDS style 5.0mm terminal. Tim's blue terminals have 5.0mm pins but the body may differ slightly.
4. **Micro-Fit pin numbers.** Follow the circuit numbers moulded into the plug housing, then check with a multimeter that pin 8 is +5V and pin 16 is ground before connecting any sensor.
5. **First flash.** A blank module may not enter flashing mode by itself: hold BOOT, tap RESET, release BOOT, then upload.
6. **No ESD protection on USB**, the same as the C3-Zero. Fine for bench flashing.

## 8. Open items

- [x] **BME280.** Not on the carrier board. The firmware already expects a BMP280 on the display node. J4 is there if one is ever wanted in the engine bay.
- [x] **Enclosure location.** Near the battery box.
- [x] **Paper test fit.** Done 2026-11-18, everything fits (section 7). L1 also passed, so the board can be ordered from `out/sensor-node-carrier-gerbers.zip`.
- [ ] **EGT probe grounding.** Measure continuity between either thermocouple lead and the probe's metal sheath. If there is continuity the MAX31855 will report "short to GND" once the probe is screwed into the manifold, and an insulated-junction probe is needed.
- [ ] **Boost sensor.** Confirm on the bench that it runs from 5V, never outputs above 5V, and whether it reads absolute or gauge pressure.
- [ ] **Module pin direction.** The MAX31855 has a right-angle header (seen in photos, 2026-11-10). The board assumes straight pins with the module lying flat. Preferred fix: swap its header for straight pins, which needs no board change. In a straight socket as it is, the module would stand on edge, held only by six pins with the thermocouple cable pulling on the top. The CAN module also has a right-angle header (Tim, 2026-11-10) and gets the same fix. Its pin order is confirmed. Modules plug into female header sockets on the carrier, with the far edge of each single-row module propped and glued.
- [x] **L1.** Measured 2026-11-18: 0.6 ohm on the meter with 0.4 ohm of lead resistance, so about 0.2 ohm (0.1 to 0.3 given the meter's one decimal place). Under the 0.5 ohm limit. At about 100 mA that is a 20 to 30 mV drop.
- [ ] **Display node TWAI pins** verified against the real wiring.
- [ ] **Trim the stray braid strand** at the MAX31855 thermocouple terminal.
- [ ] **Engine bay housing** for the sensor node, to be modelled and printed in ASA-CF. Cable entries: loom at the bottom edge, CAN and thermocouple at the right edge, 5V at the left edge, USB at the top edge.
- [ ] **Firmware changes** in section 6.

## 9. How Tim likes to work

- UK based, experienced web developer, new to PCB design and fairly new to analogue electronics. Explain electronics plainly, skip the software basics.
- Wants short, actionable answers. Uses parts he already has where possible.
- Checks things himself and will push back if something looks off.
- Never use em dashes in replies to him.

The full design doc is "Sensor Node Carrier Board" in the VW T4 Van Upgrades project on claude.ai. This file carries everything from it that the local work needs.
