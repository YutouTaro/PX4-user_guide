**Basic Assembly**

# Mounting the Flight Controller

## Orientation

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

1. [Telemetry System](#telemetry) allows to <u>play/run missions, control & monitor vehicle</u> in real time. Typically includes telemetry radios, tablet/PC and ground station software.

2. [Buzzer](#buzzer) — Provides <u>audio signals</u> that indicate what the UAV is doing

3. [Remote Control Receiver System](#rc_control) — <u>Connects to a hand-held transmitter</u> that an operator can use to manually fly the vehicle (shown is a PWM receiver with PWM->PPM converter).

4. (Dedicated) [Safety switch](#safety-switch) — Press and hold to lock and unlock motors. Only required if you are not using the recommended [GPS](#gps) with inbuilt safety switch.

5. [GPS, Compass, LED, Safety Switch](#gps) — The recommended GPS module contains GPS, Compass, LED and Safety Switch. 

6. [Power System](#power) — Powers Cube and the motor ESCs. Consists of LiPo battery, power module, and optional battery warning system (audio warning if battery power goes below a predefined level). 

> The port `GPS2` maps to `TEL4` in PX4. i.e. if connecting to `GPS2`, assign the [serial port configuration parameter](https://docs.px4.io/main/en/peripherals/serial_configuration.html) for the connected hardware to `TEL4`





## Telemetry System (Optional) <a name="telemetry"></a>

## Buzzer <a name="buzzer"></a>

## Radio Control <a name="rc_control"></a>



<a name="safety-switch"></a>

<a name="gps"></a>
<a name="power"></a>
<!--a name=""></a-->
<!--a name=""></a-->


