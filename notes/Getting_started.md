**Getting started**

# Basic Concepts

## PX4 Autopilot - Flight controller

* Controls [many different vehicle frames/types](https://docs.px4.io/main/en/airframes/airframe_reference.html), including: aircraft (multicopters, fixed wing aircraft and VTOLs), ground vehicles and underwater vehicles.
  
* [Quadrotor x](https://docs.px4.io/main/en/airframes/airframe_reference.html#quadrotor-x)
    <div class="frame_common" style="width:200px">
    <img src="../assets/airframes/types/QuadRotorX.svg"/>
    </div>

## QGroundControl - Drone code ground control station

* load (flash) PX4 onto vehicle control hardware
* setup vehicle
* change different parameters
* get real-time flight info
* create & execute fully autonomous missions

## Radio Control (RC)

* manually control the vehicle
* more topics related introduced later

## Safety Switch

* A switch that must be engaged before the vehicle can be armed.
  * for arming and disarming, see the section [below](#arming-and-disarming)
* Commonly integrated into a GPS unit
  * could also be a separated physical component

## Data/Telemetry Radios

[Data/Telemetry Radios](https://docs.px4.io/main/en/telemetry/) can provide a wireless MAVLink connection between a ground control station like *QGroundControl* and a vehicle running PX4. This makes it possible to tune parameters while a vehicle is in flight, inspect telemetry in real-time, change a mission on the fly, etc.

## Arming and Disarming <a name="arming-and-disarming"></a>

> Vehicles may have moving parts, some of which are dangerous when powered (in particular motors and propellers)!

Three power states defined by PX4:

* **Disarmed**: All motors and actuators are unpowered.
* **Prearmed**: Motors are unpowered, actuators are powered (allowing non-dangerous actuators to be bench-tested).
* **Armed:** Motors and other actuators are powered, and propellers may be spinning.
> **<u>Arm vehicles only when necessary!!!</u>**

## Heading and Directions

![Frame Heading](../assets/concepts/frame_heading.png)

![Frame Heading TOP](../assets/concepts/frame_heading_top.png)

# Sensors

## GPS & compass

> We recommend the use of an external "combined" compass/GPS module mounted as <u>far away from</u> the **motor/ESC power supply lines** as possible - typically on a pedestal or wing (for fixed-wing).

## Airspeed sensors

* Essential in detecting stall
* (For fixed-wing flight), it is the airspeed that guarantees lift not ground speed

## Tachometers

## Distance sensors

## Optical Flow sensors

# Radio Control Systems

A radio control (RC) system is required if you want to *manually* control your vehicle from a handheld transmitter.

In autonomous flight modes, remote control system is not required

> To disable RC checks, setting parameter `COM_RC_IN_MODE` to `1`

Term definition

* **Transmitter**: ground-based radio modules

* **Receiver**: vehicle-based radio modules

The number of channels the RC supports is an important quality

* channel refers to <u>switches, dials, control sticks can actually be used</u>

* at least 2 channels for ground vehicles (steering, throttle)

* at least 4 channels for aircrafts (roll, pitch, yaw, thrust)

* additional channels can be used for control other mechanisms or activate different flight modes

## Types of Remote Controls

![RC Basic Commands](../assets/flying/rc_basic_commands.png)

# PX4 Flight Modes

## Switching Between Modes

Flight modes can be transited using switches on RC or ground control station

Not all modes are available on all vehicle types, some modes behave differently on different vehicle types

In **multicopter** <u>autonomous modes</u>, RC stick movement will change to position mode by default (unless handling a critical battery failsafe)

For autonomous fixed-wing flight, stick movement is ignored

# Autonomous and Manual Modes

Term definition

* **Manual**

    * user has control over vehicle movement via RC

    * may have autopilot-assisted mechanisms to make flight easier

    * > e.g. most modes will level out the vehicle when the RC sticks are centered

    * Can be further divided into "easy" and "acrobatic" modes

    * In **easy** mode, roll and pitch sticks control vehicle angle

        * impossible to flip

    * In **acrobatic** mode, the stickes control rate of angular rotation

        * can flip

        * more maneuverable, harder to fly

  * Autonomous
  
      * fully controlled by autopilot
  
      * input from pilot/remove control is not required

Multicopter:

* Manual-Easy: [Position](https://docs.px4.io/main/en/getting_started/flight_modes.html#position-mode-mc), [Altitude](https://docs.px4.io/main/en/getting_started/flight_modes.html#altitude-mode-mc), [Manual/Stabilized](https://docs.px4.io/main/en/getting_started/flight_modes.html#manual-stabilized-mode-mc), [Orbit](https://docs.px4.io/main/en/getting_started/flight_modes.html#orbit-mode-mc)
* Manual-Acrobatic: [Acro](https://docs.px4.io/main/en/getting_started/flight_modes.html#acro-mode-mc)
* Autonomous: [Hold](https://docs.px4.io/main/en/getting_started/flight_modes.html#hold-mode-mc), [Return](https://docs.px4.io/main/en/getting_started/flight_modes.html#return-mode-mc), [Mission](https://docs.px4.io/main/en/getting_started/flight_modes.html#mission-mode-mc), [Takeoff](https://docs.px4.io/main/en/getting_started/flight_modes.html#takeoff-mode-mc), [Land](https://docs.px4.io/main/en/getting_started/flight_modes.html#land-mode-mc), [Follow Me](https://docs.px4.io/main/en/getting_started/flight_modes.html#follow-me-mode-mc), [Offboard](https://docs.px4.io/main/en/getting_started/flight_modes.html#offboard-mode-mc)

## Legend
The icons below are used within the document:

* <img src="../assets/site/remote_control.svg" width="30px" /> <a name="key_manual"></a>
    * Manual mode. Remote control required.
* <img src="../assets/site/automatic_mode.svg" width="30px" />  <a name="key_automatic"></a>
    * Automatic mode. RC control is disabled by default except to change modes. 
* <img src="../assets/site/position_fixed.svg" width="30px" /> <a name="key_position_fixed"></a>
    * Position fix required (e.g. GPS, VIO, or some other positioning system).
* <img src="../assets/site/altitude_icon.svg" width="30px" />  <a name="altitude_only"></a>
    * Altitude required (e.g. from barometer, rangefinder). 
* <img src="../assets/site/difficulty_easy.png" width="30px" /> <img src="../assets/site/difficulty_medium.png" width="30px" /> <img src="../assets/site/difficulty_hard.png" width="30px" /> <a name="key_difficulty"></a>
    * Flight mode difficulty (Easy to Hard) 


## Multicopter
### Position Mode 位置模式 <a name="position-mode-mc"></a>
[<img src="../assets/site/difficulty_easy.png" width="30px" />](#key_difficulty)&nbsp;[<img src="../assets/site/remote_control.svg" width="30px" />](#key_manual)&nbsp;[<img src="../assets/site/position_fixed.svg" width="30px" />](#key_position_fixed)

* Easy-to-fly RC mode
    * Roll and pitch sticks control acceleration over ground in vehicles forward-back and left-right directions
    * Throttle controls speed of ascent-descent
* When sticks are released/centered, the vehicle will actively brake, level and be **locked to a position**
    * The vehicle will stop when sticks are centered, rather than drifting

> Safest manual mode for new fliers

![MC Position Mode](../assets/flight_modes/position_MC.png)

## Altitude Mode 定高模式 <a name="altitude-mode-mc"></a>
[<img src="../assets/site/difficulty_easy.png" width="30px" />](#key_difficulty) [<img src="../assets/site/remote_control.svg" width="30px" />](#key_manual) [<img src="../assets/site/altitude_icon.svg" width="30px"/>](#altitude_only)

* _relatively_ easy-to-fly RC mode
    * Roll and pitch sticks control vehicle movement in the left-right and forward-back directions
    * Yaw stick controls <u>roation rate</u>
    * Throttle controls speed of ascent-descent
* When sticks are released/centered, the vehicle will level and **maintain the current altitude**.
    * The moving in the horizontal plane will continue until the momentum is dissipated by wind resistance
    * e.g. the wind could blow the aircraft to drift

> Safest _non-GPS_ manual mode for new fliers.

## Manual/Stabilized Mode 手动/自稳模式 <a name="manual-stabilized-mode-mc"></a>

* Mode
    * Roll and pitch sticks control the angle of the vehicle (attitude)
    * Yaw stick controls the rate of rotation above the horizontal plane
    * Throttle controls altitude/speed
* The control sticks will return to the center deadzone when released
    * The multicopter will level out and stop once the roll and pitch sticks are centered.
    * The vehicle will then hover in place/maintain altitude - provided it is properly balanced, throttle is set appropriately, and no external forces are applied (e.g. wind).
    * The craft will drift in the direction of any wind
        * have to control the throttle to hold altitude.

![MC Manual Flight](../assets/flight_modes/manual_stabilized_MC.png)

## Acro Mode <a name="acro-mode-mc"></a>

* RC mode for performing acrobatic maneuvers e.g. rolls and loops.
    * The roll, pitch and yaw sticks control the **rate of angular rotation** around the respective axes
    * Throttle is passed directly to the output mixer
    * When sticks are centered the vehicle will stop rotating, but **remain in its current orientation** (on its side, inverted, or whatever) and moving according to its current momentum
![MC Manual Acrobatic Flight](../assets/flight_modes/manual_acrobatic_MC_edited.png)
P.S. Up for left stick means move to the "top" direction of the vehicle, even when the vehicle is upside down 

## 


<a name="orbit-mode-mc"></a>

<a name="hold-mode-mc"></a>
<a name="return-mode-mc"></a>
<a name="mission-mode-mc"></a>
<a name="takeoff-mode-mc"></a>
<a name="land-mode-mc"></a>
<a name="follow-me-mode-mc"></a>
<a name="offboard-mode-mc"></a>







<!--
| **Icon**                                                                                                                                                                                    | **Description**                                                                                        |
|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|--------------------------------------------------------------------------------------------------------|
| <img src="../assets/site/remote_control.svg" width="30px" />                                                                                                                                | Manual mode. Remote control required.                                         |
| <img src="../assets/site/automatic_mode.svg" width="30px" />                                                                                                                                | Automatic mode. RC control is disabled by default except to change modes. <a name="key_automatic"></a> |
| <img src="../assets/site/position_fixed.svg" width="30px" />                                                                                                                                | Position fix required (e.g. GPS, VIO, or some other positioning system).                               |
| <img src="../assets/site/altitude_icon.svg" width="30px" />                                                                                                                                 | Altitude required (e.g. from barometer, rangefinder).                                                  |
| <img src="../assets/site/difficulty_easy.png" width="30px" /> <img src="../assets/site/difficulty_medium.png" width="30px" /> <img src="../assets/site/difficulty_hard.png" width="30px" /> | Flight mode difficulty (Easy to Hard)                                                                  |
-->