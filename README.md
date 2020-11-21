Overview
--------
This project is a partial port of the **ros_arduino_bridge** package to ROS 2. 
Sections of the original code that implemented support for multiple types of 
motor controllers and sensors were retained; however they have not tested 
(use at your own risk).

The stack includes a base controller for a differential drive robot that 
accepts ROS Twist messages and publishes odometry data back to the PC. The 
base controller communicates to an Arduino board through a serial interface 
to drive a motor controller and read odometry data from wheel encoders.

The primary goal of this project is to develop a differential drive robot 
based on the [Parallax Arlo Robotic Platform](https://www.parallax.com/product/arlo-complete-robot-system/) 
and that runs on ROS 2. Having said that, some effort has made to keep the 
code modular and allow for adding support for new hardware.

Features of this new ROS 2 stack include:

* Support for Arduino Uno; other Arduino boards may work but untested

* Support for the Parallax HB-25 motor controllers (used in Arlo platform)

* Default serial baud rate set to 115200

* Uses Colcon as the build system

See the original [ROS 1 ros_arduino_bridge](https://wiki.ros.org/ros_arduino_bridge) 
package if you want to back-port these motor controllers: Pololu VNH5019, 
Robogaia, L298 motor driver.

System Requirements
-------------------
1. Python 3 (tested on 3.8)
2. PySerial module. Use either:

    ```
    $ sudo pip install --upgrade pyserial

    or

    $ sudo easy_install -U pyserial
    ```

3. ROS 2 Foxy Fitzroy or later

4. Arduino IDE 1.6.6 or later. Download from https://www.arduino.cc/en/Main/Software

Preparing your Serial Port under Linux
--------------------------------------
Your Arduino will likely connect to your Linux computer as port /dev/ttyACM# or /dev/ttyUSB# where # is a number like 0, 1, 2, etc., depending on how many other devices are connected.  The easiest way to make the determination is to unplug all other USB devices, plug in your Arduino, then run the command:

    $ ls /dev/ttyACM*

or 

    $ ls /dev/ttyUSB*

Hopefully, one of these two commands will return the result you're looking for (e.g. /dev/ttyACM0) and the other will return the error "No such file or directory".

Next you need to make sure you have read/write access to the port.  Assuming your Arduino is connected on /dev/ttyACM0, run the command:

    $ ls -l /dev/ttyACM0

and you should see an output similar to the following:

    crw-rw---- 1 root dialout 166, 0 2013-02-24 08:31 /dev/ttyACM0

Note that only root and the "dialout" group have read/write access.  Therefore, you need to be a member of the dialout group.  You only have to do this once and it should then work for all USB devices you plug in later on.

To add yourself to the dialout group, run the command:

    $ sudo usermod -a -G dialout your_user_name

where your\_user\_name is your Linux login name.  You will likely have to log out of your X-window session then log in again, or simply reboot your machine if you want to be sure.

When you log back in again, try the command:

    $ groups

and you should see a list of groups you belong to including dialout. 

Installation of the ros\_arduino\_bridge stack
----------------------------------------------

    $ mkdir -p ~/dev_ws/src
    $ cd ~/dev_ws/src
    $ git clone https://github.com/benbongalon/ros_arduino_bridge.git
 
    $ colcon build --symlink-install

You should see an output similar to below: 

<pre>
Starting >>> ros_arduino_msgs
Starting >>> ros_arduino_firmware
Finished <<< ros_arduino_firmware [0.11s]                                
Finished <<< ros_arduino_msgs [0.54s]                     
Starting >>> ros_arduino_python
Finished <<< ros_arduino_python [0.70s]          

Summary: 3 packages finished [1.35s]
</pre>

Run the _setup.bash_ file to make the ros_arduino_bridge package visible from ROS.

    $ source install/setup.bash

The provided Arduino library is called ROSArduinoBridge and is
located in the ros\_arduino\_firmware package.  This sketch is
specific to the hardware requirements above but it can also be used
with other Arduino-type boards by turning off the base controller as 
described in the NOTES section at the end of this document.

To install the ROSArduinoBridge library, follow these steps:

    $ cd SKETCHBOOK_PATH

where SKETCHBOOK_PATH is the path to your Arduino sketchbook directory.

    $ cp -pr ~/dev_ws/src/ros_arduino_bridge/ros_arduino_firmware/src/libraries/ROSArduinoBridge ROSArduinoBridge

This last command copies the ROSArduinoBridge sketch files into your sketchbook folder.  The next section describes how to configure, compile and upload this sketch.


Loading the ROSArduinoBridge Sketch
-----------------------------------

* If you are using the base controller, make sure you have already installed the appropriate motor controller and encoder libraries into your Arduino sketchbook/librariesfolder.

* Launch the Arduino IDE and load the ROSArduinoBridge sketch.
  You should be able to find it by going to:

    File->Sketchbook->ROSArduinoBridge
  
NOTE: If you don't have the required base controller hardware but
still want to try the code, see the notes at the end of the file.

Choose one of the supported motor controllers by uncommenting its #define statement and ensure that others are commented out. All controllers are disabled by default.

Choose a supported encoder library by by uncommenting its #define statement and ensure that others are commented out. All encoder libraries are disabled by default.

If you want to control PWM servos attached to your controller, look for the line:

<pre>
#define USE_SERVOS
</pre>

and make sure it is not commented out like this:

<pre>
//#define USE_SERVOS
</pre>

You must then edit the include file servos.h and change the N_SERVOS
parameter as well as the pin numbers for the servos you have attached.

* Compile and upload the sketch to your Arduino.

Firmware Commands
-----------------
The ROSArduinoLibrary accepts single-letter commands over the serial port for polling sensors, controlling servos, driving the robot, and reading encoders.  These commands can be sent to the Arduino over any serial interface, including the Serial Monitor in the Arduino IDE.

**NOTE:** Before trying these commands, set the Serial Monitor baudrate to 57600 and the line terminator to "Carriage return" or "Both NL & CR" using the two pulldown menus on the lower right of the Serial Monitor window.

The list of commands can be found in the file *commands.h*.  The current list includes:

<pre>
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define GET_PID_PARAMS 'v'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define GET_PID_RATE   'y'
</pre>

For example, to get the analog reading on pin 3, use the command:

a 3

To change the mode of digital pin 3 to OUTPUT, send the command:

c 3 1

To get the current encoder counts:

e

To move the robot forward at 20 encoder ticks per second:

m 20 20

To update the values of the PID parameters _Kp, Kd, Ki, Ko_ (proportional, derivative, integral and output gains, respectively): 

u 20:12:0:20

To get the current PID parameters:

v


Testing your Wiring Connections
-------------------------------
On a differential drive robot, the motors are connected to the motor controller terminals with opposite polarities to each other.  Similarly, the A/B leads from the encoders are connected in the reverse sense to each other.  However, you still need to make sure that (a) the wheels move forward when given a positive motor speed and (b) that the encoder counts increase when the wheels move forward.

After **placing your robot on blocks**, you can use the Serial Monitor in the Arduino IDE to test both requirements.  Use the 'm' command to activate the motors, the 'e' command to get the encoder counts, and the 'r' command to reset the encoders to 0.  Remember that at the firmware level, motor speeds are given in encoder ticks per second so that for an encoder resolution of, say 4000 counts per wheel revolution, a command such as 'm 20 20' should move the wheels fairly slowly.  (The wheels will only move for 2 seconds which is the default setting for the AUTO\_STOP\_INTERVAL.)  Also remember that the first argument is the left motor speed and the second argument is the right motor speed.  Similarly, when using the 'e' command, the first number returned is the left encoder count and the second number is the right encoder count.

Finally, you can use the 'r' and 'e' commands to verify the expected encoder counts by rotating the wheels by hand roughly one full turn and checking the reported counts.


Configuring the ros\_arduino\_python Node
-----------------------------------------
Now that your Arduino is running the required sketch, you can
configure the ROS side of things on your PC.  You define your robot's
dimensions, PID parameters, and sensor configuration by editing the
YAML file in the directory ros\_arduino\_python/config.  So first move
into that directory:

    $ cd ~/dev_ws/src/ros_arduino_bridge/ros_arduino_python/config

Now copy the provided config file to one you can modify:

    $ cp arduino_params.yaml my_arduino_params.yaml

Bring up your copy of the params file (my\_arduino\_params.yaml) in
your favorite text editor.  It should start off looking like this:

```python
port: /dev/ttyUSB0
baud: 57600
timeout: 0.1

rate: 50
sensorstate_rate: 10

use_base_controller: False
base_controller_rate: 10

# === Robot drivetrain parameters
#wheel_diameter: 0.146
#wheel_track: 0.2969
#encoder_resolution: 8384 # from Pololu for 131:1 motors
#gear_reduction: 1.0
#motors_reversed: True

# === PID parameters
#Kp: 20
#Kd: 12
#Ki: 0
#Ko: 50
#accel_limit: 1.0

# === Sensor definitions.  Examples only - edit for your robot.
#     Sensor type can be one of the follow (case sensitive!):
#	  * Ping
#	  * GP2D12
#	  * Analog
#	  * Digital
#	  * PololuMotorCurrent
#	  * PhidgetsVoltage
#	  * PhidgetsCurrent (20 Amp, DC)

sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  arduino_led:          {pin: 13, type: Digital, rate: 5, direction: output}
}
```

**NOTE**: Do not use tabs in your .yaml file or the parser will barf it back out when it tries to load it.   Always use spaces instead.  **ALSO**: When defining your sensor parameters, the last sensor in the list does **not** get a comma (,) at the end of the line but all the rest **must** have a comma.

Let's now look at each section of this file.

 _Port Settings_

The port will likely be either /dev/ttyACM0 or /dev/ttyUSB0. Set accordingly.

The Arduino sketch connects at 115200 baud by default.

_Polling Rates_

The main *rate* parameter (50 Hz by default) determines how fast the
outside ROS loop runs.  The default should suffice in most cases.  In
any event, it should be at least as fast as your fastest sensor rate
(defined below).

The *sensorstate\_rate* determines how often to publish an aggregated
list of all sensor readings.  Each sensor also publishes on its own
topic and rate.

The *use\_base\_controller* parameter is set to False by default.  Set it to True to use base control (assuming you have the required hardware.)  You will also have to set the PID paramters that follow.

The *base\_controller\_rate* determines how often to publish odometry readings.

_Defining Sensors_

The *sensors* parameter defines a dictionary of sensor names and
sensor parameters. (You can name each sensor whatever you like but
remember that the name for a sensor will also become the topic name
for that sensor.)

The four most important parameters are *pin*, *type*, *rate* and *direction*.
The *rate* defines how many times per second you want to poll that
sensor.  For example, a voltage sensor might only be polled once a
second (or even once every 2 seconds: rate=0.5), whereas a sonar
sensor might be polled at 20 times per second.  The *type* must be one
of those listed (case sensitive!).  The default *direction* is input so
to define an output pin, set the direction explicitly to output.  In
the example above, the Arduino LED (pin 13) will be turned on and off
at a rate of 2 times per second.

_Setting Drivetrain and PID Parameters_

To use the base controller, you will have to uncomment and set the
robot drivetrain and PID parameters.  The sample drivetrain parameters
are for 6" drive wheels that are 11.5" apart.  Note that ROS uses
meters for distance so convert accordingly.  The sample encoder
resolution (ticks per revolution) is from the specs for the Pololu
131:1 motor.  Set the appropriate number for your motor/encoder
combination.  Set the motors_reversed to True if you find your wheels
are turning backward, otherwise set to False.

The PID parameters are trickier to set.  You can start with the sample
values but be sure to place your robot on blocks before sending it
your first Twist command.

Launching the ros\_arduino\_python Node
---------------------------------------
Take a look at the launch file arduino.launch in the
ros\_arduino\_python/launch directory.  As you can see, it points to a
config file called my\_arduino\_params.yaml.  If you named your config
file something different, change the name in the launch file.

With your Arduino connected and running the MegaRobogaiaPololu sketch,
launch the ros\_arduino\_python node with your parameters:

    $ ros2 launch ros_arduino_python arduino_launch.py

You should see something like the following output:

<details>
  <summary>HB-25 motor</summary>
<pre>
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2020-11-18-18-29-58-769592-nuc-382619
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [arduino_node.py-1]: process started with pid [382621]
[arduino_node.py-1] [INFO] [1605753004.756857299] [ubuntu_arduino]: Connected to Arduino on port /dev/ttyACM0 at 115200 baud
</pre>
</details>
 
If you have any Ping sonar sensors on your robot and you defined them
in your config file, they should start flashing to indicate you have
made the connection.

Viewing Sensor Data
-------------------
To see the aggregated sensor data, echo the sensor state topic:

    $ ros2 topic echo /arduino/sensor_state

To see the data on any particular sensor, echo its topic name:

    $ ros2 topic echo /arduino/sensor/sensor_name

For example, if you have a sensor called ir\_front\_center, you can see
its data using:

    $ ros2 topic echo /arduino/sensor/ir_front_center

You can also graph the range data using rxplot:

    $ rxplot -p 60 /arduino/sensor/ir_front_center/range


Sending Twist Commands and Viewing Odometry Data
------------------------------------------------

Place your robot on blocks, then try publishing a Twist command:

    $ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{ angular: {z: 0.5} }'

The wheels should turn in a direction consistent with a
counter-clockwise rotation (right wheel forward, left wheel backward).
If they turn in the opposite direction, set the motors_reversed
parameter in your config file to the opposite of its current setting,
then kill and restart the arduino.launch file.

**NOTE:** If your robot is heavy and reacts slowly, you may need to publish 
the command multiple times. To do so, run:

    $ ros2 topic pub /cmd_vel geometry_msgs/Twist '{ angular: {z: 0.5} }'

then hit Ctrl-C to stop sending.

Stop the robot with the command:

    $ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{}'

To view odometry data:

    $ ros2 topic echo /odom

or

   $ rxplot -p 60 /odom/pose/pose/position/x:y, /odom/twist/twist/linear/x, /odom/twist/twist/angular/z

ROS Services
------------
The ros\_arduino\_python package also defines a few ROS services as follows:

**digital\_set\_direction** - set the direction of a digital pin

    $ ros2 service call /arduino/digital_set_direction pin direction

where pin is the pin number and direction is 0 for input and 1 for output.

**digital\_write** - send a LOW (0) or HIGH (1) signal to a digital pin

    $ ros2 service call /arduino/digital_write pin value

where pin is the pin number and value is 0 for LOW and 1 for HIGH.

**servo\_write** - set the position of a servo

    $ ros2 service call /arduino/servo_write id pos

where id is the index of the servo as defined in the Arduino sketch (servos.h) and pos is the position in radians (0 - 3.14).

**servo\_read** - read the position of a servo

    $ ros2 service call /arduino/servo_read id

where id is the index of the servo as defined in the Arduino sketch (servos.h)

Using the Parallax HB-25 Motor Controller and Quadrature Encoder
----------------------------------------------------------------
The firmware supports the Parallax HB-25 motor controller and quadrature encoder. Attach an (optional) Arduino Sensor Shield to the Arduino Uno board to make the wiring connections easier.

For speed, the code is directly addressing specific Atmega328p ports and interrupts, making this implementation Atmega328p (Arduino Uno) dependent. (It should be easy to adapt for other boards/AVR chips though.)

Connect your motor controller and wheel encoders to Arduino Uno or shield as follows:

    Left wheel PWM control -- Arduino UNO Pin 8
    Right wheel PWM control -- Arduino UNO Pin 9

    Left wheel encoder A output -- Arduino UNO pin 11
    Left wheel encoder B output -- Arduino UNO pin 10

    Right wheel encoder A output -- Arduino UNO pin 13
    Right wheel encoder B output -- Arduino UNO pin 12

Make the following changes in the ROSArduinoBridge sketch:

    #define PARALLAX_HB25

Compile the changes and upload the firmware.

Using the on-board wheel encoder counters (Arduino Uno only)
------------------------------------------------------------
The firmware supports on-board wheel encoder counters for Arduino Uno.
This allows connecting wheel encoders directly to the Arduino board, without the need for any additional wheel encoder counter equipment (such as a RoboGaia encoder shield).

For speed, the code is directly addressing specific Atmega328p ports and interrupts, making this implementation Atmega328p (Arduino Uno) dependent. (It should be easy to adapt for other boards/AVR chips though.)

To use the on-board wheel encoder counters, connect your wheel encoders to Arduino Uno as follows:

    Left wheel encoder A output -- Arduino UNO pin 2
    Left wheel encoder B output -- Arduino UNO pin 3

    Right wheel encoder A output -- Arduino UNO pin A4
    Right wheel encoder B output -- Arduino UNO pin A5

Make the following changes in the ROSArduinoBridge sketch:

    /* Encoders directly attached to Arduino board */
    #define ARDUINO_ENC_COUNTER

Compile the changes and upload the firmware.

NOTES
-----
If you do not have the hardware required to run the base controller,
follow the instructions below so that you can still use your
Arduino-compatible controller to read sensors and control PWM servos.

First, you need to edit the ROSArduinoBridge sketch. At the top of
the file comment out the line:

<pre>
#define USE_BASE
</pre>

so that it looks like this:

<pre>
//#define USE_BASE
</pre>

Edit the readEncoder() and setMotorSpeed() 
    wrapper functions if using different motor controller or encoder 
    method

Compile the changes and upload to your controller.

Next, edit your my\_arduino_params.yaml file and make sure the
use\_base\_controller parameter is set to False.  That's all there is to it.
