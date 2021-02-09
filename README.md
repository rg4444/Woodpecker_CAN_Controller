# Woodpecker_CAN_Controller
Woodpecker CAN controller (OSCC)

Joystick example application designed to show how the Open Source Car Control API can be used to recieve reports from and send commands to a drive by-wire enabled vehicle.

Using an SDL2 supported game controller, inputs are normalized and converted to relative torque, throttle, and brake commands. This application also demonstrates registering callback functions to recieve and parse OSCC reports as well as vehicle state reports from the car's OBD-II CAN network.


# Pre-requisites:

- OSCC's API and firmware modules are both required, and the modules must be installed on the vehicle
- The socketcan driver for USB and PCIe CAN interfaces is required, and is pre-installed on most Linux systems
- An SDL2 supported game controller is also required, and the SDL2 library must be pre-installed
- A CAN interface adapter, such as the [Kvaser Leaf Light](https://www.kvaser.com), is also necessary in order to connect the API to the OSCC control CAN network via USB

This application has been tested with a Logitech F310 gamepad and a wired Xbox 360 controller, but should work with any SDL2 supported game controller. Controllers with rumble capabilities will provide feedback when OSCC is enabled or disabled.

[Xbox 360 Wired Controller](https://www.amazon.com/dp/B004QRKWLA)

[Logitech Gamepad F310](http://a.co/3GoUlkN)

Install the SDL2 library with the command below.

```
sudo apt install libsdl2-dev
```

# Building Joystick Commander

From the joystick commander directory, run the following sequence to build it:

```
mkdir build
cd build
cmake .. -DKIA_SOUL=ON
make
```

After initializing the CAN interface, use the channel number to start joystick commander and begin sending commands to the OSCC modules.

For example with a Kvaser Leaf Light attached, using a bitrate of 500000 and a CAN channel of 0:

```
 sudo ip link set can0 type can bitrate 500000
 sudo ip link set up can0
```

You would then run:

```
./oscc-joystick-commander 0
```

For more information on setting up a socketcan interface, check out [this guide](http://elinux.org/Bringing_CAN_interface_up).


# Controlling the Vehicle with the Joystick Gamepad

Once the joystick commander is up and running you can use it to send commands to the Arduino modules.
Use the left trigger to brake, the right trigger for throttle, and the left gamepad axis to control steering.

The vehicle will only respond to commands if control is enabled with the start button. The back button disables control.


# Application Details


### main

Entry point of joystick commander. Initializes OSCC interface, checks for controller updates in 50 ms intervals, and closes the interface when the program terminates. This contains the applications main loop.


### joystick

`joystick.c` contains the functionality necessary to initialize and interact with the game controller.


### commander

The commander files contain the joystick commander's interactivity with the OSCC API. It demonstrates opening and closing the CAN channel communications with OSCC's control CAN network, sending enable/disable commands to the modules through the API, retrieving OSCC reports through callback functions, and sending commands through the OSCC `publish` functions.


# License Information

MIT License
Please see [LICENSE.md](LICENSE.md) for more details.