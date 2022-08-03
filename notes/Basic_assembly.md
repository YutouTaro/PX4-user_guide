**Basic Assembly**

# Mounting the Flight Controller

## Orientation <a name="orientation"></a>

The flight controller should be placed on the vehicle frame **top-side up**, oriented so that the <u>heading mark arrow</u> points towards the **front** of the vehicle. 

> In the case that the heading mark arrow is not pointing to the front of the vehicle, refer to the steps to [configure the orientation](https://docs.px4.io/main/en/config/flight_controller_orientation.html) in autopilot software. 

Refer to [the section below](#Vibration-isolation) for vibration isolation.

![FC Heading Mark](../assets/qgc/setup/sensor/fc_heading_mark_1.png)

![FC Orientation](../assets/qgc/setup/sensor/fc_orientation_1.png)

# Mounting the GPS/Compass

The GPS/Compass should be mounted as **far away from other electronics** as possible, with the <u>direction marker</u> pointing towards the **front** of the vehicle. 

> The compass can be mounted in an orientation available in  _CAL\_MAGn\_ROT_.
> **WARNING** if using an orientation that not supported in  _CAL\_MAGn\_ROT_, errors/warnings will be appeared.

# Vibration Isolation<a name="Vibration-isolation"></a>

## Vibration Analysis

How to analyse the vibration of the vehicle using the logs is explained in [Log Analysis using Flight Review > Vibration](https://docs.px4.io/main/en/log/flight_review.html#vibration)

## Basic Vibration Fixes

* Make sure everything is **firmly attached** on the vehicle (landing gear, GPS mast, etc.).
* Use balanced propellers.
* Make sure to use high-quality components
  * propellers, motors, ESC and airframe. 
  * higher quality can make a big difference
* Use a vibration-isolation method to mount the autopilot. 
  * e.g. **mounting foam for FC**
  * others may have inbuilt vibration-isolation mechanisms.
* Adjust [software filters](https://docs.px4.io/main/en/config_mc/filter_tuning.html)
  * > reduce the vibration sources first, filtering them out in software is the last measure. 

# Cable wiring basics

The following basic concepts should be kept in mind when designing drone cabling:

* **High-Power** and **signal** cables should be separated as much as is practical
* Cable lengths should be the **minimum** needed to enable easy handling of wired components
  * Use cable loops for extra length
  * <u>Excess length should be avoided</u>
* The **wire tension** should be adequate to survive possible airframe deformations even in a crash landing (<u>wires must not be the first thing to break</u>)

## Signal wiring

### I2C

The [I2C bus](https://en.wikipedia.org/wiki/I%C2%B2C) is widely used for connecting **sensors**.

| Signal | Pixhawk Colors            | ThunderFly colors         | CUAV colors (I2C/CAN)     |
|:------ |:------------------------- |:------------------------- |:------------------------- |
| +5V    | ![red][rcircle] Red       | ![red][rcircle] Red       | ![red][rcircle] Red       |
| *SCL*  | ![black][blkcircle] Black | ![yellow][ycircle] Yellow | ![white][wcircle] White   |
| *SDA*  | ![black][blkcircle] Black | ![green][gcircle] Green   | ![yellow][ycircle] Yellow |
| GND    | ![black][blkcircle] Black | ![black][blkcircle] Black | ![black][blkcircle] Black |

The [Dronecode standard (opens new window)](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)assumes a 1.5k ohm pull-up resistor on SDA and SCL signals in autopilot.

#### Cable twisting

**Proper twisting of the cable wires** can greatly improve I2C bus signal cross-talk and electromagnetic compatibility. [Twisted pairs (opens new window)](https://en.wikipedia.org/wiki/Twisted_pair)

* 10 turns for each pair SCL/+5V and SDA/GND per 30cm cable length.
  ![I²C JST-GH cable](../assets/hardware/cables/i2c_jst-gh_cable.jpg)
- 4 turns of both pairs together per 30cm cable length.
  ![I²C JST-GH connector detail](../assets/hardware/cables/i2c_jst-gh_connector.jpg)

#### Pull-up resistors

Pull-up resistors are required for all ends of an I2C bus. This acts both as [signal termination (opens new window)](https://en.wikipedia.org/wiki/Electrical_termination)and as bus idle signal generator.

An oscilloscope measurement is sometimes required to check correct value of pull-up resistors. The signals on the I2C bus should have clear sharp rectangle-like edges and amplitude of few volts. In case the signal has a low amplitude, the value of pull-up resistors is too low and should be decreased. In the case of rounded signals, the value of pull-up resistors is too high.

### UAVCAN cables

[UAVCAN (opens new window)](http://uavcan.org/)is an onboard network which allows the autopilot to connect to avionics/peripherals. It uses rugged, differential signalling, and supports firmware upgrades over the bus and status feedback from peripherals.

| Signal  | Pixhawk                   | ThunderFly                | Zubax                     | CUAV (I2C/CAN)            |
| ------- | ------------------------- | ------------------------- | ------------------------- | ------------------------- |
| +5V     | ![red][rcircle] Red       | ![red][rcircle] Red       | ![red][rcircle] Red       | ![red][rcircle] Red       |
| *CAN_H* | ![black][blkcircle] Black | ![white][wcircle] White   | ![white][wcircle] White   | ![white][wcircle] White   |
| *CAN_L* | ![black][blkcircle] Black | ![yellow][ycircle] Yellow | ![yellow][ycircle] Yellow | ![yellow][ycircle] Yellow |
| GND     | ![black][blkcircle] Black | ![black][blkcircle] Black | ![black][blkcircle] Black | ![black][blkcircle] Black |

#### Cable twisting

CAN cables should also be twisted, for exactly the same reason as I2C cables.
For CAN the recommended twisting is:

- 10 turns for each pair GND/+5V and CAN_L/CAN_H per 30cm cable length.
  ![CAN JST-GH cable](../assets/hardware/cables/can_jst-gh_cable.jpg)

- 4 turns of both pairs together per 30cm cable length.

### SPI

[SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface) is synchronous serial communication interface used for connecting faster sensors and devices.
This protocol is commonly use is for connecting [optical flow](../sensor/optical_flow.md) sensors or special telemetry modems.

| Signal | Pixhawk Color             | ThunderFly color          |
| ------ | ------------------------- | ------------------------- |
| +5V    | ![red][rcircle] Red       | ![red][rcircle] Red       |
| SCK    | ![black][blkcircle] Black | ![yellow][ycircle] Yellow |
| MISO   | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| MOSI   | ![black][blkcircle] Black | ![green][gcircle] Green   |
| CS!    | ![black][blkcircle] Black | ![white][wcircle] White   |
| CS2    | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| GND    | ![black][blkcircle] Black | ![black][blkcircle] Black |

### UART

UART is used to connect peripherals to the autopilot.

The connecting cable is not crossed.
Therefore, it is necessary to connect only the autopilot and peripherals with this straight cable.
The device must cross the wiring internally by swapping RX/TX and RTS/CTS pins.

| Signal | Pixhawk Color             | ThunderFly color          |
| ------ | ------------------------- | ------------------------- |
| +5V    | ![red][rcircle] Red       | ![red][rcircle] Red       |
| TX     | ![black][blkcircle] Black | ![white][wcircle] White   |
| RX     | ![black][blkcircle] Black | ![green][gcircle] Green   |
| CTS    | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| RTS    | ![black][blkcircle] Black | ![yellow][ycircle] Yellow |
| GND    | ![black][blkcircle] Black | ![black][blkcircle] Black |

UART signals are common sources of low frequency EMI, therefore the length of the cable should be <u>minimized as much as possible</u>. Cable twisting is not needed for UART cables.

### GPS(UART) & SAFETY

[GPS receivers and magnetometers](../gps_compass/README.md) are generally very sensitive to EMI.
Therefore these should be mounted **far away from RF sources** (high-power cabling, ESCs, radio modems and its antenna).
This may be insufficient if the cabling is badly designed.

| Signal        | Pixhawk Color             | ThunderFly color          |
| ------------- | ------------------------- | ------------------------- |
| +5V           | ![red][rcircle] Red       | ![red][rcircle] Red       |
| TX            | ![black][blkcircle] Black | ![white][wcircle] White   |
| RX            | ![black][blkcircle] Black | ![green][gcircle] Green   |
| SCL           | ![black][blkcircle] Black | ![yellow][ycircle] Yellow |
| SDA           | ![black][blkcircle] Black | ![green][gcircle] Green   |
| SAFETY_SW     | ![black][blkcircle] Black | ![white][wcircle] White   |
| SAFETY_SW_LED | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| +3v3          | ![black][blkcircle] Black | ![red][rcircle] Red       |
| BUZZER        | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| GND           | ![black][blkcircle] Black | ![black][blkcircle] Black |

### GPS

| Signal | Pixhawk Color             | ThunderFly color          |
| ------ | ------------------------- | ------------------------- |
| +5V    | ![red][rcircle] Red       | ![red][rcircle] Red       |
| TX     | ![black][blkcircle] Black | ![white][wcircle] White   |
| RX     | ![black][blkcircle] Black | ![green][gcircle] Green   |
| SCL    | ![black][blkcircle] Black | ![yellow][ycircle] Yellow |
| SDA    | ![black][blkcircle] Black | ![green][gcircle] Green   |
| GND    | ![black][blkcircle] Black | ![black][blkcircle] Black |

The GPS cable connects to both the UART and I2C bus.
As twisting of UART is not applicable the length of the cable should be minimized as much as possible. 

### Analog signal (power module)

| Signal  | Pixhawk Color             | ThunderFly color          | CUAV color                |
| ------- | ------------------------- | ------------------------- | ------------------------- |
| VCC     | ![red][rcircle] Red       | ![red][rcircle] Red       | ![red][rcircle] Red       |
| VCC     | ![black][blkcircle] Black | ![red][rcircle] Red       | ![red][rcircle] Red       |
| CURRENT | ![black][blkcircle] Black | ![white][wcircle] White   | ![white][wcircle] White   |
| VOLTAGE | ![black][blkcircle] Black | ![yellow][ycircle] Yellow | ![yellow][ycircle] Yellow |
| GND     | ![black][blkcircle] Black | ![black][blkcircle] Black | ![black][blkcircle] Black |
| GND     | ![black][blkcircle] Black | ![black][blkcircle] Black | ![black][blkcircle] Black |

This connector is example of mix of relatively high-power and low voltage signaling.
Unfortunately, <u>twisting is applicable for high-power GND and VCC wires only</u>.
That does not help much for noisy reading of analog signals by autopilot.

### SAFETY

| Signal        | Pixhawk Color             | ThunderFly color          |
| ------------- | ------------------------- | ------------------------- |
| SAFE_VCC      | ![red][rcircle] Red       | ![red][rcircle] Red       |
| SAFETY_SW_LED | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| SAFETY_SW     | ![black][blkcircle] Black | ![white][wcircle] White   |
| BUZZER        | ![black][blkcircle] Black | ![blue][bluecircle] Blue  |
| +5V           | ![black][blkcircle] Black | ![red][rcircle] Red       |
| GND           | ![black][blkcircle] Black | ![black][blkcircle] Black |

## High-power wiring

General cross section requirement: <u>area of 1mm² per 8A of wire current</u>. 

A large separation between **high-power cables** and **navigation magnetometers** is always required, as the EMI from high power cabling has a significant effect on magnetometers.

While rarely practical, it is beneficial to have positive and negative wires twisted together.

### Color coding <a name="color coding"></a>

Most manufacturers use <span style="color:red">red</span>  for the high voltage line and **black** for ground.for high voltage. 

The [Pixhawk connector standard (opens new window)](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)requires only that the Voltage Common Collector (VCC) pin/cable be red.

A colour coding scheme designed for easy cable identification

* <span style="color:red">red</span> and **black** are reserved for power
* The same signal type have the same color
* * Color of the signal does not repeat in the connector for wires adjacent to each other.
    * different colors for different signal types
* Wiring harnesses with same pin count should have unique color sequence. 
  * 引脚数量的线，必须有不同的颜色序列用来区分

An example of a cable colouring:

| Color               | Name   | Preferred usage                             |
| ------------------- | ------ | ------------------------------------------- |
| ![red][rcircle]     | Red    | Power voltage                               |
| ![green][gcircle]   | Green  | General purpose signal                      |
| ![white][wcircle]   | White  | General purpose signal                      |
| ![yellow][ycircle]  | Yellow | General purpose signal                      |
| ![blue][bluecircle] | Blue   | Power return, Open-collector control signal |
| ![black][blkcircle] | Black  | GND, Power return ground                    |

<!-- references for the image source.
This approach just allows more compact markdown -->

[ycircle]: ../assets/hardware/cables/yellow.png
[rcircle]: ../assets/hardware/cables/red.png
[gcircle]: ../assets/hardware/cables/green.png
[wcircle]: ../assets/hardware/cables/white.png
[bluecircle]: ../assets/hardware/cables/blue.png
[blkcircle]: ../assets/hardware/cables/black.png

# Cube Wiring Quickstart

Using *Cube Wiring Quickstart* as example, for other quickstarts, please refer to the chapters under [Basic Assembly | PX4 User Guide](https://docs.px4.io/main/en/assembly/).

<img title="" src="../assets/flight_controller/cube/cube_black_hero.png" width="350px">

# Accessories

Cube comes with most (or all) of the accessories you will need when [purchased](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html#stores).

![Cube Accessories](../assets/flight_controller/cube/cube_accessories.jpg)

# Wiring Overview

![Cube - Wiring Overview](../assets/flight_controller/cube/cube_wiring_overview.jpg)

1. [Telemetry System](#telemetry) (**`TELEM1`**) allows to <u>play/run missions, control & monitor vehicle</u> in real time. Typically includes telemetry radios, tablet/PC and ground station software.

2. [Buzzer](#buzzer) (**`USB`**)— Provides <u>audio signals</u> that indicate what the UAV is doing

3. [Remote Control Receiver System](#rc_control) (**`RCIN`**)  — <u>Connects to a hand-held transmitter</u> that an operator can use to manually fly the vehicle (shown is a PWM receiver with PWM->PPM converter).

4. (Dedicated) [Safety switch](#safety-switch) (_`GPS1`_) — Press and hold to lock and unlock motors. Only required if you are not using the recommended [GPS](#gps) with inbuilt safety switch.

5. [GPS, Compass, LED, Safety Switch](#gps) (**`GPS1`**) — The recommended GPS module contains GPS, Compass, LED and Safety Switch. 

6. [Power System](#power) (**`POWER1`**)— Powers Cube and the motor ESCs. Consists of LiPo battery, power module, and optional battery warning system (audio warning if battery power goes below a predefined level). 

> The port **`GPS2`** maps to **`TEL4`** in PX4. i.e. if connecting to **`GPS2`**, assign the [serial port configuration parameter](https://docs.px4.io/main/en/peripherals/serial_configuration.html) for the connected hardware to **`TEL4`**

> More information about available ports can be found here: [Cube > Ports](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html#ports).

# Mount and Orient Controller

Mount the Cube as close as possible to the <u>vehicle's center of gravity</u>, ideally oriented <u>top-side up</u> and with the <u>arrow pointing towards the front</u> of the vehicle. 

> For non-default orientation, please configure the autopilot software with the orientation. [Flight Controller Orientation](#orientation).

- The Cube can be mounted using either
  
  - vibration-damping foam pads (included in the kit) or
  
  - mounting screws
    
    - the screws provided are or 1.8mm thick frameboard
    
    - customized ones can be M2.5, with thread length inside Cube range 6~7.55mm. 

![Cube Mount - Mounting Plate](../assets/flight_controller/cube/cube_mount_plate_screws.jpg)

## Telemetry System (Optional) <a name="telemetry"></a>

A telemetry system allows you to <u>communicate with</u>, <u>monitor</u>, and <u>control</u> a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **`TELEM1`** port (if connected to this port, no further configuration is required).

The other radio is connected to ground station computer/mobile device (usually via USB). 

![Telemetry Radio](../assets/flight_controller/cube/cube_schematic_telemetry.jpg)

## Buzzer <a name="buzzer"></a>

Connect to **`USB`** port, no further configuration is required

## Radio Control <a name="rc_control"></a>

A remote control (RC) radio system is required for control the vehicle *manually*. 

Instructions of how to connect different types of receivers. 

### PPM-SUM / Futaba S.Bus receivers

ground (-), power (+), and signal (S) to the RC pins (**`RCIN`**)

![Cube - RCIN](../assets/flight_controller/cube/cube_rc_in.jpg)

"Spektrum Satellite Receivers" and "PWM Receivers", skipped

## Safety Switch <a name="safety-switch"></a>

Only required when not using the recommended GPS (which has a inbuilt safety switch)

When flying without a GPS, must attach the switch directly to the **`GPS1`** port in order to arm the vehicle and fly. 

## GPS + Compass + Safety Switch + LED <a name="gps"></a>

- Recommended GPS modules: *Here* and *Here+*
  
  - incorporate a GPS module, Compass, Safety Switch and LEDs
  
  - Here+ supports centimeter level positioning via RTK

- The module should be 
  
  - mounted on the frame as <u>far away from other electronics </u>as possible, 
  
  - with the <u>direction marker towards the front </u>of the vehicle
  
  - connected to the **`GPS1`** port using the supplied <u>8-pin cable</u>.

- The safety switch is integrated with the GPS module, and enabled by default 
  
  - To disable, press and hold the safety switch for 1 second. 
  
  - You can press the safety switch again to enable safety and disarm the vehicle
  
  a schematic view of the module and its connections
  
  ![Here+ Connector Diagram](../assets/flight_controller/cube/here_plus_connector.png)

## Power<a name="power"></a>

Cube is powered from a Lithium Ion Polymer (LiPo) Battery via a Power Module. 

The battery is connected to the **`POWER1`** port.

The power module supplies the board, and may separately supply power to electronic speed controllers (ESCs).

A typical power setup for a multicopter vehicle:

![Power Setup - MC](../assets/flight_controller/cube/cube_wiring_power_mc.jpg)

> The power (+) rail of **MAIN/AUX** is *not powered* by the power module supply to the flight controller. In order to drive servos for rudders, elevons, etc., it will need to be separately powered.
> 
> This can be done by connecting the power rail to a BEC equipped ESC, a standalone 5V BEC, or a 2S LiPo battery. Ensure the voltage of servo you are going to use is appropriate!

## SD Card (optional)

SD card is needed to log and analyse flight details, run missions, and to use UAVCAN-bus hardware. 

The SD card slot can be found as shown.

![Cube - Mount SDCard](../assets/flight_controller/cube/cube_sdcard.jpg)

## Motors

Motors/servos are connected to the **MAIN** and **AUX** ports in the order specified for your vehicle in the [Airframe Reference](https://docs.px4.io/main/en/airframes/airframe_reference.html).

> If the frame is not listed in the reference, a "generic" airframe of the correct type can be used. 

> **Make sure the correct mapping is used!!!** 
> 
> The mapping is not consistent across frames (e.g. throttle for different frames is on different outpu

For [quadrotor x](https://docs.px4.io/main/en/airframes/airframe_reference.html#quadrotor-x)

* **MAIN1**: motor 1
* **MAIN2**: motor 2
* **MAIN3**: motor 3
* **MAIN4**: motor 4
