# Getting started

## Basic Concepts

### PX4 Autopilot - Flight controller

* Controls [many different vehicle frames/types](https://docs.px4.io/main/en/airframes/airframe_reference.html), including: aircraft (multicopters, fixed wing aircraft and VTOLs), ground vehicles and underwater vehicles.
  
  * [Quadrotor x](https://docs.px4.io/main/en/airframes/airframe_reference.html#quadrotor-x)
    
    <div class="frame_common" style="width:200px">
    <img src="../assets/airframes/types/QuadRotorX.svg"/>
    </div>

### QGroundControl - Dronecode ground control station

* load (flash) PX4 onto vehicle control hardware

* setup vehicle

* change different parameters

* get real-time flight inAfo

* create & execute fully autonomous missions

### Radio Control (RC)

* manually control the vehicle

* more topics related introduced later

### Safety Switch

* A switch that must be engaged before the vehicle can be armed.
  
  * for arming and disarming, see the section [below](#arming-and-disarming)

* Commonly integrated into a GPS unit
  
  * could also be a separated physical component

### Data/Telemetry Radios

[Data/Telemetry Radios](https://docs.px4.io/main/en/telemetry/) can provide a wireless MAVLink connection between a ground control station like *QGroundControl* and a vehicle running PX4. This makes it possible to tune parameters while a vehicle is in flight, inspect telemetry in real-time, change a mission on the fly, etc.

### Arming and Disarming <a name="arming-and-disarming"></a>

> Vehicles may have moving parts, some of which are dangerous when powered (in particular motors and propellers)!

Three power states defined by PX4:

* **Disarmed**: All motors and actuators are unpowered.

* **Prearmed**: Motors are unpowered, actuators are powered (allowing non-dangerous actuators to be bench-tested).

* **Armed:** Motors and other actuators are powered, and propellers may be spinning.

> **<u>Arm vehicles only when necessary!!!</u>**

### Heading and Directions

![Frame Heading](../assets/concepts/frame_heading.png)

![Frame Heading TOP](../assets/concepts/frame_heading_top.png)

## Sensors

### GPS & compass

> We recommend the use of an external "combined" compass/GPS module mounted as <u>far away from</u> the **motor/ESC power supply lines** as possible - typically on a pedestal or wing (for fixed-wing).

### Airspeed sensors

* Essential in detecting stall

* (For fixed-wing flight), it is the airspeed that guarantees lift not ground speed

### Tachometers

### Distance sensors

### Optical Flow sensors

## Radio Control Systems

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
  
  

### Types of Remote Controls

![RC Basic Commands](../assets/flying/rc_basic_commands.png)


