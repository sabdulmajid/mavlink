# MAVLink

## Code explanation

The `mavgen.py` is a Python script that generates a Python implementation of a MAVLink protocol XML file. MAVLink is a protocol for communication between unmanned aerial vehicles (UAVs) and ground control stations. The XML files that define the MAVLink protocol specify the names, types, and sizes of the messages that can be sent between UAVs and ground control stations.

The Python script works by first parsing the MAVLink XML files. This is done using the mavparse module, which is a Python library for parsing MAVLink XML files. Once the XML files have been parsed, the script generates a Python implementation of the MAVLink protocol. The generated Python implementation includes classes for each of the messages defined in the MAVLink XML files. The classes provide methods for sending and receiving messages, as well as methods for getting and setting the properties of the messages.

The Python script also validates the MAVLink XML files. This is done by comparing the XML files to the MAVLink schema, which is a XML document that defines the syntax of MAVLink XML files. If the XML files do not conform to the schema, the script will print an error message.

The Python script is a useful tool for developers who want to create applications that communicate with UAVs using the MAVLink protocol. The script can be used to generate Python implementations of MAVLink XML files, which can then be used to send and receive messages between UAVs and ground control stations.

Here is a more detailed explanation of the steps that the Python script takes to generate a Python implementation of a MAVLink protocol XML file:

1. The script imports the mavparse module.
2. The script parses the MAVLink XML file using the `mavparse.MAVXML` class.
3. The script creates a Python class for each message defined in the MAVLink XML file.
4. The script adds methods to the Python classes for sending and receiving messages.
5. The script adds methods to the Python classes for getting and setting the properties of the messages.
6. The script validates the MAVLink XML file against the MAVLink schema.
7. The script prints an error message if the `MAVLink XML` file does not conform to the schema.
8. The script returns the Python classes.

After that, a function called `mavgen_python_dialect()` is defined. This function generates Python code for a MAVLink dialect. The function takes three arguments:

`dialect`: The name of the MAVLink dialect.

`wire_protocol`: The MAVLink wire protocol version.

`with_type_annotations`: A boolean value indicating whether to generate type annotations for the Python code.

The function first gets the path to the MAVLink dialect directory. It then gets the path to the MAVLink dialect XML file. If the XML file does not exist, the function tries to get the path to the XML file from the message definitions directory.

The function then creates an Opts object. This object contains the following information:

The output directory.
The MAVLink wire protocol version.
The language to generate the Python code in.
Whether to generate type annotations for the Python code.
The function then checks if the Python 2 or Python 3 version of the mavgen script is available. It then calls the mavgen script to generate the Python code.

The function finally returns a boolean value indicating whether the Python code was generated successfully.

The mavgen script is a Python script that generates Python code for a MAVLink dialect. The script takes two arguments:

`output`: The output directory.
`xml`: The path to the MAVLink dialect XML file.

The script first parses the MAVLink dialect XML file. It then generates Python code for each message defined in the XML file. The generated Python code includes classes for each message. The classes provide methods for sending and receiving messages, as well as methods for getting and setting the properties of the messages.

The script finally writes the generated Python code to the output directory.

## Spec Details

Pixhawk 4 is a powerful flight controller that can be used to control a variety of unmanned aerial vehicles (UAVs). It is based on the STM32F405RGT6 microcontroller and includes a number of features that make it ideal for UAV applications, such as:

- 32-bit processor
- 1MB of Flash memory
- 512KB of RAM
- 100 PWM outputs
- 16 ADC inputs
- 1 IMU
- 1 GPS
- 1 barometer
- 1 power management unit

MAVLink is a lightweight messaging protocol that is commonly used to communicate between UAVs and ground stations. It is a simple and efficient protocol that is well-suited for use in UAV applications.

The Raspberry Pi 3 is a single-board computer that can be used for a variety of tasks, including controlling UAVs. It is based on the Broadcom BCM2835 ARM Cortex-A53 processor and includes a number of features that make it ideal for UAV applications, such as:

- Quad-core 1.2GHz processor
- 1GB of RAM
- 40-pin GPIO header
- 4 USB ports
- 1 HDMI port
- 1 Ethernet port
- 1 Wi-Fi adapter
- 1 Bluetooth adapter

To use MAVLink and a Raspberry Pi 3 to control a Pixhawk 4, you will need to install the following software:

- MAVLink C library
- MAVLink Python library
- Raspberry Pi OS

Once you have installed the software, you can connect the Pixhawk 4 to the Raspberry Pi 3 using a serial cable. You can then use the MAVLink libraries to send commands to the Pixhawk 4 and receive data from it.

Here are some examples of how you can use MAVLink and a Raspberry Pi 3 to control a Pixhawk 4:

- Send a command to the Pixhawk 4 to arm it.
- Send a command to the Pixhawk 4 to takeoff.
- Send a command to the Pixhawk 4 to land.
- Receive data from the Pixhawk 4 about its current state, such as its altitude, speed, and heading.

## Set-up a connection

The 'mavutil' module provides a function, `mavutil_connection()` which always a user to connect with a device. Example code to connect to a UDP port:

```python
from pymavlink import mavutil
connection = mavutil.mavlink_connection('udpin:localhost:14540')
connection.wait_heartbeat()
print("Heartbeat received from the system (system %u component %u)" % (connection.target_system, connection.target_component))
```

`udpin` creates a socket to listen for a UDP connection, whereas, `udpout` creates a socket that iniatiates a UDP connection.
`wait_heartbeat()` waits for a heartbeat message from the system. This is a blocking function, so it will wait until a heartbeat is received.
Other useful `mavlink_connection()` parameters: `source_system (default 255)`, `source_component (default 0)` and `dialect (default ArduPilot)`

As for receiving messages, the following code shows an example of how to do so efficiently:

```python
try:
  altitude = connection.messages['GPS_RAW_INT'].alt
  timestamp = connection.messages['GPS_RAW_INT'].time_usec # or timestamp = connection.time_since('GPS_RAW_INT')
  print("Altitude: %s" % altitude)
  print("Timestamp: %s" % timestamp)
except:
  print("No GPS_RAW_INT message received")
```

There is another way to receive messages automatically, which waits for and intercepts any messages as they arrive:

```python
def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
  '''
    Receive the next MAVLink message that matches the given type and condition
    • type:        Message name(s) as a string or list of strings - e.g. 'SYS_STATUS'
    • condition:   Condition based on message values - e.g. 'SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4'
    • blocking:    Set to wait until message arrives before method completes.
    • timeout:     ? <!-- timeout for blocking message when the system will return. Is this just a time? -->
  '''
```

It is always good practise to check that a message is valid before using it:

```python
  msg = m.recv_match(type='SYS_STATUS', blocking=True)
  if not msg:
    return
  if msg.get_type() == "BAD_DATA":
    if mavutil.all_printable(msg.data):
      sys.stdout.write(msg.data)
      sys.stdout.flush()
  else:
    #Message is valid
    # Use the attribute
    print('Mode: %s' % msg.mode)
```

## Heartbeats

```python
def heartbeat_send(self, type, autopilot, base_mode, custom_mode, system_status, mavlink_version=3, force_mavlink1=False):
    '''
    The heartbeat message shows that a system is present and responding.
    The type of the MAV and Autopilot hardware allow the
    receiving system to treat further messages from this
    system appropriate (e.g. by laying out the user
    interface based on the autopilot).

    • type              : Type of the MAV (quadrotor, helicopter, etc.) (type:uint8_t, values:MAV_TYPE)
    • autopilot         : Autopilot type/class (type:uint8_t, values:MAV_AUTOPILOT)
    • base_mode         : System mode bitmap (type:uint8_t, values:MAV_MODE_FLAG)
    • custom_mode       : A bitfield for use for autopilot-specific flags (type:uint32_t)
    • system_status     : System status flag (type:uint8_t, values:MAV_STATE)
    • mavlink_version   : MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version (type:uint8_t)
    '''
```

As for an example on how to send a heartbeat:

  ```python
    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0) # GCS stands for Ground Control Station
      # Or use the following method if it is from a MAVLink application
      connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
  ```

## Save and Load Key & Last-Timestamp

```python
import mavutil

def save_key_and_timestamp(self):
  # Saves the secret key and last timestamp to permanent storage and returns: None
  if self._secret_key is None:
    raise ValueError("secret_key must be set")

  with open("key.txt", "wb") as f:
    f.write(self._secret_key)

  with open("timestamp.txt", "wb") as f:
    f.write(self._initial_timestamp)

def load_key_and_timestamp(self):
  # Loads the secret key and last timestamp from permanent storage and returns: None
  try:
    with open("key.txt", "rb") as f:
      self._secret_key = f.read()

    with open("timestamp.txt", "rb") as f:
      self._initial_timestamp = f.read()
  except FileNotFoundError:
    self._secret_key = None
    self._initial_timestamp = None
```

## Signing
Signing in mavutil refers to the process of adding a cryptographic signature to a message. This signature can be used to verify the authenticity and integrity of the message.

When a message is signed, the sender uses their secret key to generate a signature. This signature is then attached to the message. When the message is received, the receiver uses the sender's public key to verify the signature. If the signature is valid, then the receiver can be confident that the message was sent by the claimed sender and that the message has not been tampered with.

Here is the code for signing/unsigning a message:

  ```python
  #Setup signing
  def setup_signing(self, secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=None)

  # Disable signing (clear secret key and all the other settings specified with setup_signing)
  def disable_signing(self):
  ```

Mavutil also allows for unsigned_callback, the example code for that being:

  ```python
  connection = mavutil.mavlink_connection('udpin:localhost:14540')

  def unsigned_callback(self, msg):
    if msgId == mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS:
      return True
    else:
      return False
    
  # Pass the callback function to the mavlink connection
  secret_key = chr(42) * 32
  connection.setup_signing(secret_key, sign_outgoing=True, allow_unsigned_callback=unsigned_callback)
  ```

## Useful Examples
  - [apmsetrate.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/apmsetrate.py)
  - [bwtest.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/bwtest.py)
  - [magtest.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/magtest.py)
  - [mav2pcap.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mav2pcap.py)
  - [mav_accel.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mav_accel.py)
  - [mav_replay_estimator.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mav_replay_estimator.py)
  - [mavgps.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mavgps.py)
  - [mavtcpsniff.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mavtcpsniff.py)
  - [mavtest.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mavtest.py)
  - [mavtester.py](https://github.com/ArduPilot/pymavlink/blob/2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965/examples/mavtester.py)

## MAVLink 2.0 New Features
Nothing much is different from MAVLink 1.0 to 2.0, but here are some of the new features:

  1. **24 bit message ID** - Allows over 16 million unique message definitions in a dialect (MAVLink 1 was limited to 256)
  2. **Packet signing** - Authenticate that messages were sent by trusted systems.
  3. **Message extensions** - Add new fields to existing MAVLink message definitions without breaking binary compatibility for receivers that have not updated.
  4. **Empty-byte payload truncation** - Empty (zero-filled) bytes at the end of the serialized payload must be removed before sending (All bytes were sent in MAVLink 1, regardless of content).
  5. **Compatibility Flags/Incompatibility Flags** - Allow for backwards compatible evolution of the protocol by indicating frames that must be handled in a special/non-standard way (packets with compatibility flags can still be handled in the standard way, while packets with incompatibility flags must be dropped if the flage is not supported).

## MAVLink, Pixhawk 4, and ArduPilot
Here is a really good guide on how to integrate & communicate between MAVLink & Raspberry Pi 3 [will be of great use for the project]:
[Communication between RP & MAVLink](https://www.linkedin.com/pulse/communication-between-drone-raspberry-pi-via-mavlink-yan-pang/)

## Open Drone ID (ODID)
Useful information parsed from this website: [ODID](https://mavlink.io/en/services/opendroneid.html)

There are 4 methods for broadcasting Open Drone ID messages:
  1. Bluetooth Legacy Advertising (Bluetooth 4.x)
  2. Bluetooth Long Range with Extended Advertising (Bluetooth 5.x)
  3. Wi-Fi Neighbor-aware Network (Wi-Fi NaN)
  4. Wi-Fi Beacon (vendor specific information element in the SSID beacon frame)

Example uses of this:
  1. A flight controller sends ID, location etc. data to an onboard Bluetooth/Wi-Fi RID transmitter component.
  2. An onboard Bluetooth/Wi-Fi receiver picks up drone ID messages from surrounding aircraft, relays this information using MAVLink drone ID messages to the flight controller, which then uses the information e.g. for Detect And Avoid (DAA) calculations.
  3. A drone sends MAVLink drone ID messages via its control link to the Ground Control Station (GCS). The GCS is connected via the Internet to a Remote ID server, which stores and publishes the drone's ID, location etc.
  4. As above but in the other direction for DAA calculations.
  5. A Remote ID Display application (RID) on the GCS listens to all drone ID data received from surrounding UAs and displays their position to the operator.

## PixHawk 2.4.8 Protocol Matching

The Pixhawk flight controller supports both MAVLink 1.0 and MAVLink 2.0. The default protocol is MAVLink 1.0, but it can be changed to MAVLink 2.0 by setting the MAV_PROTO_VERSION parameter.

Here are some additional details about the two MAVLink protocol versions:
 - MAVLink 1.0 is the original MAVLink protocol. It is a simple and efficient protocol that is well-suited for embedded systems.
 - MAVLink 2.0 is a newer, more powerful version of the MAVLink protocol. It is designed to be more secure, extensible, and efficient than MAVLink 1.0.

If you are using a Pixhawk flight controller, you can use either MAVLink 1.0 or MAVLink 2.0. The choice of which protocol to use depends on your specific needs. If you are looking for a simple and efficient protocol, then MAVLink 1.0 is a good choice. If you need a more secure, extensible, and efficient protocol, then MAVLink 2.0 is a better choice.

## MAVLink Messages (In-depth)
Here is a list of all useful messages to interact with the MAVLink device:
  - **OPEN_DRONE_ID_BASIC_ID**: Provides an ID for the UA, characterizes the type of ID and identifies the type of UA.
  - **OPEN_DRONE_ID_LOCATION**: Provides location, altitude, direction, and speed of the UA.
  - **OPEN_DRONE_ID_AUTHENTICATION**: Provides authentication data for the UA.
  - **OPEN_DRONE_ID_SELF_ID**: Optional plain text message that can be used by operators to identify themselves and the purpose of an operation. Can also be used to provide optional additional clarification in an emergency/remote ID system failure situation.
  - **OPEN_DRONE_ID_SYSTEM**: Includes the operator location/altitude, multiple aircraft information (group/swarm, if applicable), full timestamp and possible category/class information.
  - **OPEN_DRONE_ID_OPERATOR_ID**: Provides the operator ID.
  - **OPEN_DRONE_ID_MESSAGE_PACK**: A payload mechanism for combining the messages above into a single message pack. Used with Bluetooth Extended Advertising, Wi-Fi NaN and Wi-Fi Beacon.
  - **OPEN_DRONE_ID_ARM_STATUS**:	Sent by RID transmitter/receiver components to indicate that the RID system is "ready to use". This should be used as an arming condition for the flight stack. Note that this differs from the HEARTBEAT which indicates that the component is "alive" but not necessarily ready to use.
  - **OPEN_DRONE_ID_SYSTEM_UPDATE**:	A subset of the OPEN_DRONE_ID_SYSTEM message, containing only the fields that must be updated at a high rate. Typically sent from the GCS to provide data to the RID transmitter component. If both OPEN_DRONE_ID_SYSTEM and OPEN_DRONE_ID_SYSTEM_UPDATE are used, the more efficient OPEN_DRONE_ID_SYSTEM_UPDATE will be used at a high rate and the full OPEN_DRONE_ID_SYSTEM at a low rate, to reduce the traffic on the control link.

Note: [American Society for Testing and Materials (ASTM) Regulations](https://en.wikipedia.org/wiki/ASTM_International) are all met by each of these commands

## Android Support for MAVLink Drone Location
There is a very neat open source OpenDroneID Android Receiver application developed that could be of great use for the project. It is available on GitHub:
[OpenDroneID Android Receiver](https://github.com/opendroneid/receiver-android)

## Message Update Rates
ASTM F3411 and ASD-STAN prEN 4709-002 standards both require that the LOCATION message (the message that contains the drone's location) be sent atleast once per second. The rest of these messages must be broadcasted atleast once every 3 seconds (note: BASIC ID and SYSTEM messages are required to be sent atleast once every second)

## Routing Drone ID Messages (Inside the Unmanned Aircraft System (UAS))

General overview: [Diagrammatic View](https://mavlink.io/assets/opendroneid/conceptual_overview.png)

## MAVLink Components
 - **MAV_COMP_ID_AUTOPILOT1**: The flight controller/autopilot. Knows the ID of the UA, the current location, altitude, speed, direction, attitude (roll, pitch, and yaw), battery voltage, GPS status, flight mode, and environment (weather conditions, obstacles, etc.)
 - **Ground Control Station**: GCS with a human user interface for inputting the operator ID, text description of the flight purpose, method for obtaining the operator location,video feed from the UAV, flight control, data logging, mission planning, payload control, and telemetry.
 - **MAV_COMP_ID_ODID_TXRX_1**: A Remote ID transmitter/receiver component (Bluetooth/Wi-Fi/Internet)
 - **MAV_COMP_ID_ODID_TXRX_2**: A Remote ID transmitter/receiver component (Bluetooth/Wi-Fi/Internet)
 - **MAV_COMP_ID_ODID_TXRX_3**: A Remote ID transmitter/receiver component (Bluetooth/Wi-Fi/Internet)

## Common Protocols UAVs (such as the PixHawk 2.4.8)
 - **MAV_CMD_DO_SET_MODE**: This protocol is used to set the UAV's flight mode. The flight mode determines how the UAV will behave, and there are a variety of different flight modes available
 - **MAV_CMD_DO_CHANGE_ALT**: This protocol is used to change the UAV's altitude. The altitude can be changed in a variety of ways, including up, down, and to a specific altitude
 - **MAV_CMD_DO_MOTOR_TEST**: This protocol is used to test the UAV's motors. The motors can be tested individually or all at once
 - **MAV_CMD_NAV_TAKEOFF**: This protocol is used to take off the UAV. The UAV will take off vertically and reach a specified altitude
 - **MAV_CMD_NAV_LAND**: This protocol is used to land the UAV. The UAV will land vertically and come to a stop
 - **MAV_CMD_MISSION_START**: This protocol is used to start a mission. The mission is a pre-planned flight path that the UAV will follow
 - **MAV_CMD_MISSION_PAUSE**: This protocol is used to pause a mission. The UAV will hold its current position and altitude until the mission is resumed
 - **MAV_CMD_MISSION_END**: This protocol is used to end a mission. The UAV will return to its home position and altitude

There are a couple of features not added yet to the OpenDroneID protocol that could be useful for many projects (you can consider contributing to this project by adding these features):

 - Starting/stopping broadcast
 - Configure the broadcast method (BT4, BT5, Beacon, NaN)
 - Wi-Fi channel configuration for Beacon
 - Message update rates on the air

## DroneKit Software In The Loop (SITL) Simulator

### Quick Starting the SITL Simulator
To run the latest version of Copter:
```shell
dronekit-sitl copter
```
SITL will then start and wait for TCP connections on ```127.0.0.1:5760```

You can specify a particular vehicle and version, and also parameters like the home location, the vehicle model type (e.g. “quad”), etc. For example:
```shell
dronekit-sitl plane-3.3.0 --home=-35.363261,149.165230,584,353
```

There are a number of other useful arguments:
```shell
dronekit-sitl -h            #List all parameters to dronekit-sitl.
dronekit-sitl copter -h     #List additional parameters for the specified vehicle (in this case "copter").
dronekit-sitl --list        #List all available vehicles.
dronekit-sitl --reset       #Delete all downloaded vehicle binaries.
dronekit-sitl ./path [args...]  #Start SITL instance at target file location.
```

My first two tasks are to:
 1. Mission to travel between two points
 2. Configuring different sensors and access their data

For future reference when simulating: 
  - [DroneKit-SITL documentation](https://github.com/dronekit/dronekit-sitl)
  - [Running examples with DroneKit](https://dronekit-python.readthedocs.io/en/latest/examples/running_examples.html). Some useful examples include:
    - [flightreplay.py](https://github.com/dronekit/dronekit-python/blob/master/examples/flight_replay/flight_replay.py)
    - **[drone_delivery.py](https://github.com/dronekit/dronekit-python/blob/master/examples/drone_delivery/drone_delivery.py)**
      -  Code demonstrates how to use DroneKit to communicate with a simulated vehicle and perform a simple delivery mission. The steps are: start server, process POST request, connect to the vehicle, and close simulator.
    - **[performance_test.py](https://github.com/dronekit/dronekit-python/blob/master/examples/performance_test/performance_test.py)**
      - Code demonstrates how to use DroneKit to communicate with a simulated vehicle and perform a simple delivery mission. The steps are: start connection, create message listener, send message, log, and close object.
    - [simple_goto.py](https://github.com/dronekit/dronekit-python/blob/master/examples/simple_goto/simple_goto.py)
    - [vehicle_state.py](https://github.com/dronekit/dronekit-python/blob/master/examples/vehicle_state/vehicle_state.py)
  - [GitHub Repository with useful examples](https://github.com/dronekit/dronekit-python/tree/master/examples)
  
## QuickStart Guide
- Open a new terminal (I'm using MacOS 14.3, with Python 3.11.3)
- Get the dronekit-sitl package by running the following command: ```dronekit-sitl copter```
- Open a new terminal and run the following command to enter the directory: ```cd /Users/ayman/Desktop/Ayman/UAV```
- Enter the virtual environment by running the following command: ```source myenv/bin/activate```
- Get into the myenv directory ```cd myenv/python_code``` and then the run script that is needed ```python script.py```


For a Raspberry Pi SSH:
- SSH into the Raspberry Pi by running the following command: ```ssh ayman@192.168.0.107``` in a new terminal
- Exit the SSH (but still stay in the terminal) by running the following command: ```exit```

Reasons for using a virtual environment: 
- Isolate different projects from each other. This helps to prevent conflicts between different projects, and it also makes it easier to manage the dependencies of each project
- Use different versions of Python for different projects. This can be useful if you are working on a project that requires a specific version of Python, but your system has a different version installed
- Keep your system Python environment clean. Virtual environments allow you to install Python packages in a separate directory, so they don't pollute your system Python environment
- Install packages without administrator privileges. You may not have access to install packages on your system, or you may not want to install packages globally
- Share your project requirements with others. You can easily share your project's requirements by giving them the requirements.txt file

## SSH-ing into the Raspberry Pi
- Find Raspberry Pi's IP address (when connected to the Internet)
  - In the Raspberry Pi's terminal, run the following command ```hostname -I```
- SSH into the Raspberry Pi
  - In the terminal, run the following command ```ssh ayman@192.168.0.107```
  - Enter the password (default is ```raspberry```); no need for this now since I have created an SSH Key-Pair for smoother login
- (Optional) Add new user to sudo users: ```sudo usermod newuser_name -a -G pi,adm,dialout,cdrom,sudo,audio,video,plugdev,games,users,input,netdev,spi,i2c,gpio``` then ```sudo visudo``

It's as simple as that! Now you can easily access the Raspberry Pi's terminal from your computer, remotely. 
Extra links to look at if stuck: [Guide 1: SSH Guide for Raspberry Pi](https://www.instructables.com/Connect-Raspberry-pi-to-MacOS-or-Linux-using-SSH/) or [Guide 2: SSH-ing into Raspberry Pi from Mac](https://medium.com/@thedyslexiccoder/how-to-remotely-access-a-raspberry-pi-on-a-mac-via-ssh-be285d418f54) or [Guide 3: Connecting Raspberry Pi to Pixhawk via MAVLink](https://hackmd.io/@willy541222/RasPi-pixhawk)


## Breakdown of Various Software Packages
Here's a breakdown of the different components and how they come into play:
1. **MAVProxy**: MAVProxy is a command-line-based ground control station software that acts as a communication gateway between your Python script and the drone. It provides a flexible and extensible interface for sending and receiving MAVLink messages, monitoring telemetry, and controlling the drone. MAVProxy can run on your computer and establish a connection with the simulated drone (DroneKit-SITL) or a physical drone.

2. **DroneKit-SITL**: DroneKit-SITL is a software-in-the-loop (SITL) simulator that emulates the behavior of a physical drone. It allows you to run and test your drone control scripts in a simulated environment without the need for a physical drone. DroneKit-SITL integrates with MAVProxy, providing the simulated drone's telemetry data and receiving commands from MAVProxy.

3. **Mission Planner**: Mission Planner is a ground control station software primarily developed for the ArduPilot autopilot system. It offers a feature-rich graphical interface for mission planning, monitoring, and controlling the drone. Mission Planner is designed to communicate directly with the drone's autopilot, such as Pixhawk, through a telemetry link or a direct connection.

4. **QGroundControl**: QGroundControl is another popular ground control station software that supports multiple autopilot systems, including ArduPilot and PX4. Like Mission Planner, it provides a user-friendly interface for mission planning, monitoring, and controlling the drone.

## Connecting Drone to Software
When trying to connect to a drone, there are chances that you will walk into problems. Here is a list of the most common problems:
1. The Tx and Rx signal wires are connected wrong
2. The companion software is not correctly loaded
3. Serial interface is not enabled (in the Raspberry Pi)
4. Forgetting the confirguration of the bt overlay  in the ```/boot/config.txt``` file
5. Incorrect MAVProxy statement
6. Incorrect MAVProxy baud rate
7. Failure to set up the flight controller protocol
8. Failure to use to the correct communication port
9. Dysfunctional or dirty Raspberry Pi GPIO pins
  1. GPIO pins can be checked using pigpio. To install pigpio, run the following command: ```sudo apt-get install pigpio python-pigpio python3-pigpio```
10. Loose GPIO connection
11. Loose or bad telemetry cable
12. Dysfunctional flight controller or UART pin

## Camera Options for Drone
1. **Sony RX0 II**: This compact and lightweight camera offers excellent image quality, 4K video recording, and a high frame rate for capturing fast-moving objects. It can be easily integrated with OpenCV.
2. **FLIR Duo Pro R**: If thermal imaging is essential for your UAV's autonomous functions, this camera combines a thermal sensor with a visible light sensor. It is suitable for applications like object detection and tracking in challenging environments.
3. **DJI Zenmuse X7**: Designed specifically for drone use, this camera offers outstanding image quality with a Super 35mm sensor and supports interchangeable lenses. It provides excellent control and stability, making it a reliable option for autonomous UAVs.
4. **GoPro HERO9 Black**: A popular choice among drone enthusiasts, the HERO9 Black offers 5K video recording, superb image stabilization, and a rugged design. Its compact size and compatibility with OpenCV make it a versatile option.
5. **FLIR Blackfly S**: This camera series includes various models with different resolutions, frame rates, and interfaces, providing flexibility based on your specific needs. They offer high-quality imaging and can be integrated with OpenCV.
6. **Raspberry Pi Camera Module**: If you're looking for a more budget-friendly option, the Raspberry Pi Camera Module is a popular choice. It is compatible with OpenCV and offers decent image quality and flexibility.

## QGroundControl
QGroundControl is an open-source ground control station software that supports multiple autopilot systems, including ArduPilot and PX4. It provides a user-friendly interface for mission planning, monitoring, and controlling the drone.
Here is how to setup the drone using QGroundControl:
  1. Download QGroundControl from [here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
  2. Connect the drone to the computer using a USB cable
  3. Open QGroundControl
  4. Go to the top left corner and click on the QGroundControl icon
  5. Click on the Vehicle Setup option
  6. Click on the Wizard option
  7. Select the appropriate vehicle type
  8. Select the appropriate airframe
  9. Select the appropriate flight controller
  10. Select the appropriate communication port
  11. Select the appropriate baud rate
  12. Click on the Finish button
  13. Click on the OK button
  14. Click on the Reboot Vehicle button
  15. Click on the OK button

## Mission Planner on Linux
Mission Planner is not really for use on Linux, but there is a method to force install it through the Windows ```.exe``` file. Here are the steps to running it on a virtual Linux machine, after following the installation guide here: [installation guide for MONO and Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html)

After installing, follow these steps to get it running on the virtual machine:
1. Open VirtualBox
2. Click on the virtual machine you want to run Mission Planner on (in our case, the Ubuntu 22.04.2 LTS)
3. Click on the Start button
4. Resize the window to suit your liking
5. Voila! You can now run Mission Planner on Linux

## Setting up the Simulation
1. Open a new terminal and run the following command: ```dronekit-sitl copter```
2. Open a new terminal and run the following command: ```mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551```
3. Enter Mission Planner on the Virtual Machine, and enter the local port number ```14551```
4. Open another terminal and run any scripts needed, such as the following command: ```python3 altitude_change.py```. Will most likely have to ```cd``` into a specific directory first
5. Watch the simulated drone fly!

## OpenCV Models for Drone
There are several good OpenCV models for object detection that can be used with drone imagery. OpenCV itself provides various algorithms and techniques for object detection, such as Haar cascades, HOG (Histogram of Oriented Gradients), and DNN (Deep Neural Networks) models. These models can be trained on a separate dataset or can be found pre-trained on popular object detection benchmarks.

These are a few pre-trained object detection models that work well with OpenCV:
1. **YOLO (You Only Look Once)**: YOLO is a popular real-time object detection framework known for its speed and accuracy. You can find pre-trained YOLO models, such as YOLOv3 or YOLOv4, which are trained on large-scale datasets like COCO (Common Objects in Context) or VOC (Visual Object Classes). These models are widely used in drone applications for detecting objects in real-time.
2. **SSD (Single Shot MultiBox Detector)**: SSD is another widely-used object detection framework known for its speed and accuracy. It provides real-time detection of objects across various scales and aspect ratios. Pre-trained SSD models, such as SSD300 or SSD512, are available and can be used with OpenCV for drone-based object detection.
3. **Faster R-CNN (Region-based Convolutional Neural Networks)**: Faster R-CNN is a two-stage object detection framework that offers high accuracy. It employs a region proposal network (RPN) to generate potential object regions and then performs classification and bounding box regression on those regions. While Faster R-CNN models are generally slower compared to YOLO and SSD, they can provide better accuracy. You can find pre-trained Faster R-CNN models, such as ResNet-Faster-RCNN or Inception-Faster-RCNN, which can be integrated with OpenCV.

## Basic Overview So Far

Setting up the Development Environment (iMac):
1. **Install Python**: Download and install Python from the official website (https://www.python.org) if you haven't already.
2. **Install necessary libraries**: Use pip (Python's package manager) to install the required packages such as pymavlink, dronekit, opencv-python, and numpy.
3. **Connecting to the UAV**: Establish a connection between your computer and the UAV using appropriate hardware (e.g., telemetry module or USB cable). Ensure that your drone is compatible with MAVLink and has the necessary firmware installed.
4. **Interacting with the UAV using MAVLink**: Use the pymavlink library in Python to communicate with the UAV via MAVLink. You can send commands, receive telemetry data, and monitor the UAV's status. Refer to the MAVLink documentation for details on message types and communication protocols.
5. **Implementing Control Logic**: Develop Python scripts to define your UAV's behavior and control logic. This could include mission planning, autonomous flight, manual control, and other functionalities. Use MAVLink messages to send commands and receive telemetry data as required.
6. **Integrating QGroundControl**: QGroundControl is a popular ground control station for MAVLink-enabled UAVs. Use it to monitor and control your UAV during flight operations. Connect QGroundControl to your UAV using the appropriate connection method (e.g., telemetry module or USB cable).
7. **Integrating OpenCV**: OpenCV is a computer vision library that can be used for various tasks, such as image processing, object detection, and visual tracking. Incorporate OpenCV into your project to add computer vision capabilities to your UAV system. For example, you can perform image analysis on the video feed from the UAV's camera.
8. **Image Processing with OpenCV**: Use OpenCV to process the video stream from the UAV's camera. You can perform tasks like object detection, image stabilization, optical flow, and more. Implement the desired algorithms and techniques based on your project requirements.

## Setting up the Development Environment Using a Virtual Machine (Linux & Mission Planner):
1. **Set up a Linux Virtual Machine**: Install a virtualization software such as Oracle VirtualBox or VMware on your computer. Create a new virtual machine and install a Linux distribution of your choice (e.g., Ubuntu, CentOS, etc.) as the guest operating system.
2. **Install Mission Planner**: Within the Linux virtual machine, download the Mission Planner software from the official website (https://ardupilot.org/planner/). Mission Planner is primarily designed for Windows, but it can also run on Linux using Mono, an open-source implementation of Microsoft's .NET framework.
3. **Install Mono**: Mono is required to run .NET applications on Linux. Open a terminal in the Linux virtual machine and install Mono by following the instructions provided by the Mono project (https://www.mono-project.com/). Make sure you install the necessary dependencies and configure the environment properly.
4. **Launch Mission Planner**: Once Mono is installed, navigate to the directory where you downloaded Mission Planner and run the executable file using the Mono runtime. In the terminal, enter the following command: ```mono MissionPlanner.exe```. This will launch Mission Planner within the Linux virtual machine.
5. **Connect to the UAV**: Connect your UAV to the computer running the Linux virtual machine using the appropriate hardware (e.g., telemetry module or USB cable). Ensure that the UAV has the necessary firmware and is compatible with Mission Planner.
6. **Interact with the UAV**: Use Mission Planner within the Linux virtual machine to communicate with and control the UAV. You can perform tasks such as mission planning, waypoint navigation, monitoring telemetry data, and more.

## Setting Home Location & Simulating in Mission Planner
1. Ensure that the simulation is running in DroneKit-SITL and the connection to MAVProxy is established.
2. Open Mission Planner and connect to the simulated vehicle.
3. In the top menu, click on "Config" and then select "Planner" from the drop-down menu.
4. In the Planner Configuration window, go to the "Initial Setup" tab.
5. Under "Home Location," you will find the "Set" button next to the latitude and longitude fields. Click on "Set."
6. A dialog box will appear where you can manually enter the latitude and longitude coordinates of your desired home location. Enter the coordinates and click "OK" to set the home location.

## Setting Waypoints & Simulating in Mission Planner
1. Ensure that the simulation is running in DroneKit-SITL and the connection to MAVProxy is established.
2. In Mission Planner, ensure that you are connected to the simulated vehicle.
3. Go to the "Flight Plan" tab in Mission Planner.
4. On the left side, you will see a map interface. Right-click on the map at the location where you want to set your destination point.
5. A menu will appear with various options. Select "Add WP (at current alt)" to add a waypoint at that location.
6. Repeat step 4 to add additional waypoints if desired, forming a route from your home location to the destination point.
7. Once you have added all the waypoints, you can click the "Upload" button to upload the mission to the simulated vehicle.
8. After the mission is uploaded, you can arm the vehicle and start the simulation by following the appropriate steps in Mission Planner.

## (Optional) Setting up SITL with Linux Using ArduPilot
Here is a link on how to use the ArduPilot SITL software; including installation and setup guides: [Guide to ArduPilot's SITL App](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

## Switching Back To Mac For Drone Simulation Development
Since I've run into numerous errors trying to install and use Mission Planner, I have decided to revert back to trying to just use the iMac to try and get code working on it. I will be using the DroneKit-SITL software to simulate a drone on the iMac. Here is a link on how to use the DroneKit-SITL software; including installation and setup guides: [Guide to DroneKit-SITL](https://dronekit-python.readthedocs.io/en/latest/develop/sitl_setup.html)

1. **Launch the SITL simulation**: in a terminal, run the following command: ```dronekit-sitl copter --home=28.382731,36.482608,0,180```. This will start the SITL simulation for a quadcopter.
2. **Connect to SITL using MAVProxy**: in another terminal, run the following command: ```mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:10.55.222.120:14550```. This command connects to the SITL simulation and creates two output connections: one for GCS (Ground Control Station) and another for additional applications.
3. **Verify the connection**: in the terminal, you should see the MAVProxy console, indicating a successful connection to SITL.
4. **Run your simulation or interact with the vehicle using pymavlink or DroneKit**: run the python script in another terminal and try to get it to interact with the simulated vehicle.

Note: You can use either pymavlink or DroneKit to interact with the simulated vehicle. DroneKit provides a higher-level API and additional functionality, while pymavlink offers a lower-level interface.

## Location To Use
I have selected these two points to run simulations on to try and work. Have currently run into issues with pymavlink and hence will have to research into another way to run simulations for the drone project.
- Point 1: (28.382731,36.482608,0,20) [in the form of (lat,lon,alt,yaw)]
- Point 2: (28.382503,36.482018,0,20) [in the form of (lat,lon,alt,yaw)]

## Alternatives To Mission Planner (& PyMAVLink)
After setting almost everything up, I ran into some issues with pymavlink. Apparently pymavlink is not compatible with Python 3.10 or 3.11, but I also had issues trying to install it on Python 3.6 & 3.7. And in the end I was unable to find a fix for that. Now I am looking into alternatives to Mission Planner and pymavlink. I have found a few alternatives that could potentially be of use, and [here is a link](https://discuss.ardupilot.org/t/pymavlink-vs-mavsdk-python-vs-dronekit-python-for-udp-receiving-program/86422) discussing them.

## Settings To Note When Setting Things Up Again
- When trying to run the Python script, you have to ```cd``` into the directory where the script is located, so for me ```cd ~/Desktop/python_code```
- Before running the Python script on the terminal, you have to change some lines of code on the dronekit ```__init__.py``` file, located at ```/usr/local/lib/python3.10/dist-packages/dronekit/``` for me. You then go in and change the ```import collections``` line to ```import collections.abc as collections```. This is because the dronekit library is not compatible with Python 3.10, and this is a temporary fix for it.


Notes: I run into this error when trying to run the Python script (and I am guessing that this because of the pymavlink issue):
```bash
Connecting to vehicle on: tcp:127.0.0.1:5760
WARNING:dronekit:Link timeout, no heartbeat in last 5 seconds
ERROR:dronekit.mavlink:Exception in MAVLink input loop
Traceback (most recent call last):
  File "/usr/local/lib/python3.10/dist-packages/dronekit/mavlink.py", line 211, in mavlink_thread_in
    fn(self)
  File "/usr/local/lib/python3.10/dist-packages/dronekit/__init__.py", line 1371, in listener
    raise APIException('No heartbeat in %s seconds, aborting.' %
dronekit.APIException: No heartbeat in 30 seconds, aborting.
Traceback (most recent call last):
  File "/home/ayman/Desktop/python_code/test.py", line 33, in <module>
    vehicle = connect(connection_string, wait_ready=True)
  File "/usr/local/lib/python3.10/dist-packages/dronekit/__init__.py", line 3167, in connect
    vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)
  File "/usr/local/lib/python3.10/dist-packages/dronekit/__init__.py", line 2276, in initialize
    raise APIException('Timeout in initializing connection.')
dronekit.APIException: Timeout in initializing connection.
```

## Open Solo 4
After finding out that DroneKit is a 'dead' API and project now, I had to search for alternatives for it. I have stumbled upon Open Solo 4, which is a fork of the original Solo project. It is a Linux-based operating system for the Solo drone, and it is open-source. It is also compatible with Python 3.10, which is a plus. Here is the general installation overview of installing the IMX Firmware via SSH/SCP:
1. Download the latest Open Solo 4 firmware from [here](https://github.com/OpenSolo/OpenSolo/wiki/Install-via-SSH)
2. Extract the downloaded file
3. Connect to the Solo's WiFi network (you will want to do the copter first, since you will lose connectivity when the controller update initiates)

As for SSH-ing into the Solo (for a Copter IMX):
1. SSH into the copter with ```IP 10.1.1.10```, username ```root```, password ```TjSDBkAu``` (this is the default password)
2. ```sololink_config --update-prepare sololink``` cleans up and prepares the directories
3. Copy 3dr-solo.tar.gz and 3dr-solo.tar.gz.md5 to the /log/updates directory on the copter
4. ```sololink_config --update-apply sololink --reset``` executes the update and reboots

## Quickstart Guide for MAVSDK
MAVSDK is a cross-platform SDK for communicating with drones. It is written in C++ and has bindings for Python, Swift, and Rust. It is compatible with Python 3.7+. Here is a quickstart guide for MAVSDK:
1. Install MAVSDK-Python: ```pip3 install mavsdk```
2. Install the lightweight REPL (Read-Eval-Print-Loop) tool called asyncio: ```pip3 install aioconsole```
3. After the SITL is ready, we can open a REPL and start writing code for the drone.
4. Then run the following code (line-by-line) int the REPL:
```python
from mavsdk import System
drone = System()
await drone.connect()
await drone.action.arm()
await drone.action.takeoff()
# Extra commands to try out to see how the drone responds:
  # await drone.action.land()
  # await drone.action.disarm()
  # await drone.action.kill()
  # await drone.action.reboot()
  # await drone.action.return_to_launch()
  # await drone.action.goto_location(47.398039859999997, 8.5455725400000002, 10, 0)
```
## Useful Examples (for MAVSDK)
  - [Simple Taking Off & Landing Example Code](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/takeoff_and_land.py)
  - [Mission Simulation From Point to Point to Point](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/mission.py)
  - [Calibration Code](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/calibration.py)

## Fixing 'Permission Denied' Errors
When trying to install dependencies and all, I found out that there were a lot of times where I couldn't download a specific package because I didn't have the permission to do so.
My quick fix for this is to run the following command in the terminal window that is installing all of this: ```sudo chown -R $(whoami) $(brew --prefix)/*```

## Guide to Start Arducopter-SITL
It is a long and tedious method, but to get the ArduCopter SITL running, I had to follow the following commands:

1. ```cd /Users/ayman/Desktop/Ayman/ardupilot```
2. ```git submodule update --init --recursive```
3. ```./waf configure --board sitl```
4. ```./waf copter --upload```
5. ```cd build/sitl/bin```
6. ```./arducopter --model quad ../../../../Tools/autotest/default_params/copter.parm```

And as for connecting the simulated drone to the SITL:
1. ```cd /Users/ayman/Desktop/Ayman/UAV/myenv/python_code/UAV-Flight-Code/``` then ```python3 testing_ping_connection.py``` 
2. OR directly just type this into a terminal ```/usr/bin/python3 /Users/ayman/Desktop/Ayman/UAV/myenv/python_code/UAV-Flight-Code/testing_ping_connection.py```

## Different Types of Remote Connections
1. **SSH (Secure Shell)**: SSH is a cryptographic network protocol that provides secure remote access to systems over an unsecured network. It enables you to establish a secure command-line or terminal connection to a remote server. SSH encrypts the data exchanged between the client and the server, ensuring confidentiality and integrity.

2. **Remote Desktop Protocol (RDP)**: RDP is a proprietary protocol developed by Microsoft. It allows you to remotely access and control a Windows-based computer over a network connection. With RDP, you can view the remote desktop and interact with it as if you were physically present at the machine.

3. **VNC (Virtual Network Computing)**: VNC is a graphical desktop sharing system that enables remote control of a computer's desktop environment. It works on the client-server model, where the server sends a compressed image of the remote desktop to the client, which in turn sends user input back to the server. VNC is platform-independent and supports various operating systems.

4. **Telnet**: Telnet is an older remote connection protocol that allows you to establish a command-line connection with a remote system. It operates over a network and provides a text-based interface for remotely accessing and managing systems. However, Telnet doesn't provide encryption, making it less secure compared to SSH.

5. **FTP (File Transfer Protocol)**: FTP is a standard network protocol used for transferring files between a client and a server on a computer network. It provides a way to access, modify, and transfer files remotely. FTP can be used with a command-line interface or various FTP client software that provides a graphical interface.

6. **SFTP (SSH File Transfer Protocol)**: SFTP is a secure alternative to FTP that uses the SSH protocol for file transfer. It provides secure file transfer capabilities and remote file management similar to FTP. SFTP encrypts the data during transmission, ensuring confidentiality.

7. **SCP (Secure Copy)**: SCP is another secure file transfer protocol that works over SSH. It allows you to securely copy files between remote systems. SCP provides both encryption and authentication, ensuring secure file transfers.

## General Steps to Connect to Drone
1. **Select a wireless communication method**: There are several options available, depending on your requirements and the range you need. Common options include Wi-Fi, radio telemetry, and cellular data. Choose a method that suits your needs and the capabilities of your hardware.

2. **Set up the ground station**: On the ground, you'll need a computer or a mobile device to act as your ground station. Ensure it has the necessary hardware and software to communicate with the Pixhawk 4 and Raspberry Pi 4. This typically involves installing ground control station software, such as Mission Planner, QGroundControl, or APM Planner.

3. **Establish the wireless link**: Depending on the chosen communication method, you will need to make a secure connection between them. Options are Wi-Fi, radio telemetry, and cellular data.

4. **Configure the software**: Launch the ground control station software on your ground station computer. Connect to the Pixhawk 4 using the appropriate connection option within the software. This may involve selecting the correct serial port or Wi-Fi network, depending on your setup. Once connected, you should be able to access the telemetry data and control the drone.

5. **Monitor the connection**: Keep an eye on the wireless connection between the drone and the ground station during flight. Ensure the signal strength remains strong, and there are no interruptions or interference. Some ground control station software provides telemetry data, including the link quality, which can help you monitor the connection health.

## How To Stay Connected In The Air
1. **Set up the Pixhawk 4**: Connect your Pixhawk 4 to your Raspberry Pi 4 using a USB cable. Ensure that the Pixhawk 4 is powered on and recognized by the Raspberry Pi 4. You can verify this by checking the device connection using the ```ls /dev/serial/by-id``` command in the terminal (on the RP4). Make a note of the serial port name assigned to the Pixhawk 4 (```/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00```).

2. **Launch SITL (Software In The Loop)**: SITL allows you to simulate the Pixhawk's behavior and test your software without an actual drone. Launch the SITL environment by opening a terminal window and running the following command:

```arduino
dronekit-sitl copter --home=latitude,longitude,altitude,yaw
```
Replace latitude, longitude, altitude, and yaw with the desired starting position and orientation for the simulated drone.

3. **Connect QGroundControl**: Launch QGroundControl on your ground station computer. Ensure that your Wi-Fi connection is active and select the appropriate Wi-Fi network on both your ground station computer and the Raspberry Pi 4. QGroundControl should automatically detect the Pixhawk 4 and establish a connection.

4. **Start MAVProxy**: Open a new terminal window on your Raspberry Pi 4 and run the following command to start MAVProxy:

```python
mavproxy.py --master=/dev/serial/by-id/your-pixhawk-serial-port --out udp:ip-of-ground-station:14550 --out udp:ip-of-ground-station:14551
```
Replace your-pixhawk-serial-port with the serial port name of your Pixhawk 4, and ip-of-ground-station with the IP address of your ground station computer running QGroundControl. This command establishes a connection between the Pixhawk 4 and QGroundControl via MAVProxy.

5. **Interact with the drone using MAVProxy**: With MAVProxy running, you can enter commands in the MAVProxy terminal to interact with the simulated drone. MAVProxy provides a command-line interface for controlling the drone and accessing telemetry data. You can arm/disarm the drone, take off, control its movements, and perform other operations. Refer to MAVProxy documentation for a list of available commands.

6. **Develop with pymavlink and dronekit-sitl**: You can use pymavlink and dronekit-sitl libraries to develop custom scripts and applications for interacting with the drone. These libraries provide APIs to send and receive MAVLink messages, control the drone's behavior, and access telemetry data. You can write Python scripts using these libraries to automate tasks, perform custom missions, or integrate additional functionality.

## Connecting a Raspberry Pi 4 to Pixhawk 4 (physical connection)

1. Physical Connection: Connect the Pixhawk's telemetry port (usually TELEM1 or TELEM2) to the Raspberry Pi's serial port using appropriate cables. The Pixhawk's telemetry port uses a serial protocol like UART or MAVLink.

2. Software Setup: Install the required software on your Raspberry Pi to communicate with the Pixhawk. You'll need to install the MAVProxy or DroneKit-Python libraries, which provide APIs for interacting with the Pixhawk.

   - MAVProxy: Install MAVProxy by running the following command in the terminal:
     ```
     sudo pip3 install MAVProxy
     ```

   - DroneKit-Python: Install DroneKit-Python by running the following command:
     ```
     sudo pip3 install dronekit
     ```

3. Code Execution: Once the software is installed, you can write Python scripts to communicate with the Pixhawk and control the flight. Here's an example script to get started:

   ```python
   from dronekit import connect, VehicleMode

   # Connect to the Pixhawk
   connection_string = '/dev/ttyAMA0'
   vehicle = connect(connection_string, wait_ready=True, baud=57600)

   # Arm and takeoff
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True
   vehicle.simple_takeoff(10)  # Replace 10 with your desired altitude in meters

   # Do some flight operations...

   # Disconnect from the Pixhawk
   vehicle.close()
   ```

   This example script connects to the Pixhawk, arms it, performs a guided takeoff to an altitude of 10 meters, and then you can add your own flight control logic as needed.

4. Running Code When Raspberry Pi Restarts: If you want your code to run automatically when the Raspberry Pi is turned on or restarted, you can add your script to the system's startup process. You can achieve this by modifying the Raspberry Pi's systemd service.

   - Create a new systemd service unit file by running the command:
     ```
     sudo nano /etc/systemd/system/myflightcontrol.service
     ```

   - Add the following content to the file:
     ```
     [Unit]
     Description=My Flight Control
     After=network.target

     [Service]
     ExecStart=/usr/bin/python3 /path/to/your/script.py

     [Install]
     WantedBy=multi-user.target
     ```

     Replace `/path/to/your/script.py` with the actual path to your Python script.

   - Save the file and exit the text editor.

   - Enable the service to start on boot:
     ```
     sudo systemctl enable myflightcontrol.service
     ```

   - Reboot the Raspberry Pi:
     ```
     sudo reboot
     ```

   After the reboot, your code will automatically start running.

Please note that connecting a Raspberry Pi to a Pixhawk flight controller and controlling an aircraft involves potential safety risks. It's crucial to have a good understanding of flight operations and safety precautions. Always follow local regulations and ensure you have the necessary knowledge and experience to handle the system safely.

## Log After 1st Test
```
AP: Frame: QUAD/X
Received 973 parameters (ftp)
Saved 973 parameters to mav.parm
Flight battery 80 percent
AP: PreArm: EKF attitude is bad
AP: PreArm: AHRS: Waiting for 3D fix
AP: PreArm: Battery 1 below minimum arming voltage
GPS lock at 780 meters
LOITER> Mode LOITER
LAND> Mode LAND
STABILIZE> LOITER> Mode LOITER
LAND> LOITER> STABILIZE> Mode STABILIZE
LAND> Mode LAND
STABILIZE> Mode STABILIZE
AP: PreArm: Battery 1 below minimum arming voltage
AP: EKF3 IMU0 origin set
AP: EKF3 IMU0 is using GPS
AP: EKF3 IMU1 origin set
AP: EKF3 IMU1 is using GPS
LOITER> Mode LOITER
LAND> LOITER> STABILIZE> Mode STABILIZE
Flight battery 80 percent
AP: PreArm: Battery 1 below minimum arming voltage
AP: PreArm: Battery 1 below minimum arming voltage
LOITER> Mode LOITER
LAND> STABILIZE> Mode STABILIZE
``x`ff~x~`ffxfx`fx`f``xx`xfx``f`x`x`fx```~`x`~~x``~x`~xx`~x`~~ff`fxfx`f`~`~``~fff`f``f`~xffx~``~xxxx~~xf~`~~xx``f`~`f`f~fx`fx`xx`xx`~f`ff~fx`fx`xx`x``f`ff`fx`fffff~fx`~~~ff``~`fxx``x`ffxxf`~`fx~x~```~f`f~f`f~~~f`~``xf`~ff`ffxxfx``fxff`fxfx`ff~f`~f`~~~`~x~f`~fff~``~xfx`xxx``f``~f~ff`~f``~xfx~xx``xx~xx``f`x`xxf`x`f`f~fx`f``x``~f`f``~x`~x`xxxxff`f~fx`fxx~~~xx`f`f`ff~xxff`fxfx`f`~``f`fx``f`f~fx`f~ffx```xxff`~``ff~`~``f~ff~`xxf`f~fx`f``f~f`~~ff~`xxxffx~x`xff~f`f~fx`fffx`fff`f`xf`f~fx`f`xfx~```x`x`f~x`fxx~f~~fx~f``~~``f`f~fx````~fx`ff~x~f`f~````~pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyAMA0
Connect /dev/ttyAMA0 source_system=255
Failed to load module: No module named 'cmdlong'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Failed to load module: No module named 'terrain'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Log Directory: 
Telemetry log: mav.tlog
Waiting for heartbeat from /dev/ttyAMA0
MAV> GPS lock at 784 meters
Detected vehicle 1:1 on link 0
online system 1
STABILIZE> Mode STABILIZE
fence present
fence enabled
AP: ArduCopter V4.3.3 (34e8e02c)
AP: ChibiOS: 66e5de0d
AP: fmuv3 002A0032 3339510A 34383938
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X

STABILIZE> AP: PreArm: Battery 1 below minimum arming voltage
mode GUIDEDReceived 973 parameters (ftp)
Saved 973 parameters to mav.parm
Flight battery 70 percent
AP: GPS Glitch or Compass error

STABILIZE> Got COMMAND_ACK: DO_SET_MODE: ACCEPTED
GUIDED> Mode GUIDED
`x`xff`xx``fxxxx`~`fxx`x`~~`f`xx`f`ff~`x`~`~~ffxxff~`f~f`~`~`fffxx`f`xfx`f~`f~``~ff``f~~`ff~`f~x`x~ffxff`f```~ffxxff```x~xfxx`f`xxfx~fx`fx~fff`f~~f`~f``x`x`ff```xf~````f``ffx`xf~~mavproxy.py --master=/dev/ttyAMA0
Connect /dev/ttyAMA0 source_system=255
Failed to load module: No module named 'cmdlong'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Failed to load module: No module named 'terrain'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Log Directory: 
Telemetry log: mav.tlog
Waiting for heartbeat from /dev/ttyAMA0
MAV> GPS lock at 784 meters
Detected vehicle 1:1 on link 0
online system 1
GUIDED> Mode GUIDED
fence present
fence enabled
AP: ArduCopter V4.3.3 (34e8e02c)
AP: ChibiOS: 66e5de0d
AP: fmuv3 002A0032 3339510A 34383938
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X
AP: Glitch cleared
Received 973 parameters (ftp)
Saved 973 parameters to mav.parm
Flight battery 70 percent
AP: PreArm: Battery 1 below minimum arming voltage
AP: GPS Glitch or Compass error
AP: Glitch cleared
AP: PreArm: Battery 1 below minimum arming voltage
AP: GPS Glitch or Compass error
AP: EKF variance
Flight battery 70 percent
AP: PreArm: GPS and AHRS differ by 26.2m
AP: PreArm: Battery 1 below minimum arming voltage
AP: EKF3 lane switch 1
AP: EKF3 lane switch 1
AP: PreArm: GPS glitching
AP: PreArm: Battery 1 below minimum arming voltage
Flight battery 70 percent
AP: PreArm: GPS and AHRS differ by 10.3m
AP: PreArm: Battery 1 below minimum arming voltage
AP: PreArm: GPS glitching
AP: PreArm: Battery 1 below minimum arming voltage
f`f`x`fxxf`ffxf~~`xx`~``xf~~`f```~xffx`~f``~fxxx``~f`fff~``xffffffx`~fx`x~xf`x```~xff```~x~`xx```fxxf`~`~```~f`f``f~x~````~`f~~`xx``x```x`~~f`~f`~fx`x``ff``xf``xxf``fxfx`~``xx`x~xxx``````f`f~`~`xfx`x`~x`~`~~x`f~f`~ffxf`ff~fxx~~x`f`f``f``~xf`f`x``fxfxff~xx`x~pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyAMA0
Connect /dev/ttyAMA0 source_system=255
Failed to load module: No module named 'cmdlong'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Failed to load module: No module named 'terrain'. Use 'set moddebug 3' in the MAVProxy console to enable traceback
Log Directory: 
Telemetry log: mav.tlog
Waiting for heartbeat from /dev/ttyAMA0
MAV> Detected vehicle 1:1 on link 0
online system 1
GUIDED> Mode GUIDED
fence present
fence enabled
GPS lock at 784 meters
AP: ArduCopter V4.3.3 (34e8e02c)
AP: ChibiOS: 66e5de0d
AP: fmuv3 002A0032 3339510A 34383938
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 8.0kHz/1.0kHz
AP: Frame: QUAD/X
mode SReceived 973 parameters (ftp)
Saved 973 parameters to mav.parm
Flight battery 70 percent
AP: PreArm: GPS glitching
AP: PreArm: Battery 1 below minimum arming voltage
arm check gps
GUIDED> Unknown command 'marm check gps'
arm check gps
GUIDED> AP: EKF3 lane switch 1
AP: EKF variance
AP: PreArm: GPS and AHRS differ by 56.2m
AP: PreArm: Battery 1 below minimum arming voltage
AP: EKF variance

```

## Getting Nowhere With Tests
After working on this project for multiple weeks now, we aren't able to make any progress because of the incompatibility of DroneKit with everything else, such as Python 3.11, pyMAVlink and even the inability to connect with the MAVProxy instance in another terminal. I am still looking for bug fixes, but cannot find anything since the software is not maintained at all. My last resort now is to FULLY start from scratch, and write code in Python 2.7 and use all of the old dependencies of DroneKit. This is a very time consuming process, but I am willing to do it if it means that I can get this project to work. Hopefully I get somewhere.

## Yarn Lint Issues
```typescript
ayman@iMac-shkhsy amelia-mobile % yarn lint
yarn run v1.22.19
$ eslint .
can't resolve reference #/definitions/directiveConfigSchema from id #
can't resolve reference #/definitions/directiveConfigSchema from id #
can't resolve reference #/definitions/directiveConfigSchema from id #
can't resolve reference #/definitions/directiveConfigSchema from id #
Warning: React version not specified in eslint-plugin-react settings. See https://github.com/jsx-eslint/eslint-plugin-react#configuration .

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/__tests__/Components/LockScreen-test.js
  22:31  warning  'initialLoadingValue' is defined but never used  @typescript-eslint/no-unused-vars
  35:31  warning  'initialLoadingValue' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/App.tsx
  1:17  warning  'useEffect' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/BottomSheet.tsx
  24:44  warning  'updateProfile' is defined but never used       @typescript-eslint/no-unused-vars
  27:18  warning  'updateRes' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/DoubleButtonModal.tsx
  30:32  warning  'Common' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/InAppContext.tsx
  5:15  warning  'data' is defined but never used  @typescript-eslint/no-unused-vars
  9:15  warning  'user' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/MainProfile/Tags.tsx
  15:11  warning  'Gutters' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/NotificationCards/NotifCards.tsx
    3:10  warning  'updateInfo' is defined but never used          @typescript-eslint/no-unused-vars
  303:10  warning  'readNotif' is assigned a value but never used  @typescript-eslint/no-unused-vars
  719:16  warning  'hour' is defined but never used                @typescript-eslint/no-unused-vars
  720:18  warning  'minute' is defined but never used              @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/NotificationCards/SocialCards.tsx
  3:10  warning  'updateInfo' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/NotificationCards/SystemCards.tsx
  3:10  warning  'updateInfo' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/NotificationCards/notifHooks.tsx
   2:8   warning  'React' is defined but never used                @typescript-eslint/no-unused-vars
   5:10  warning  'useGetCommentsQuery' is defined but never used  @typescript-eslint/no-unused-vars
   7:10  warning  'getUserById' is defined but never used          @typescript-eslint/no-unused-vars
  47:10  warning  'message' is assigned a value but never used     @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/OnboardingProfile/ProfileHead.tsx
  14:42  warning  'title' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/OnboardingProfile/ProfileTail.tsx
   3:10  warning  'Params' is defined but never used                  @typescript-eslint/no-unused-vars
  21:19  warning  'width' is assigned a value but never used          @typescript-eslint/no-unused-vars
  33:24  warning  'interestRes' is assigned a value but never used    @typescript-eslint/no-unused-vars
  34:26  warning  'professionRes' is assigned a value but never used  @typescript-eslint/no-unused-vars
  35:24  warning  'headlineRes' is assigned a value but never used    @typescript-eslint/no-unused-vars
  36:19  warning  'bioRes' is assigned a value but never used         @typescript-eslint/no-unused-vars
  96:7   warning  'styles' is assigned a value but never used         @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/OnboardingProfile/Tags.tsx
  16:11  warning  'width' is assigned a value but never used   @typescript-eslint/no-unused-vars
  16:18  warning  'height' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/ProfileSettings/ConnectSocialsLine.tsx
  3:53  warning  'View' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/ProfileSettings/InputField.tsx
  3:22  warning  'Text' is defined but never used   @typescript-eslint/no-unused-vars
  3:39  warning  'View' is defined but never used   @typescript-eslint/no-unused-vars
  5:11  warning  'Props' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/ProfileSettings/ProfileSettingsLine.tsx
  5:11  warning  'Props' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/ProfileSettings/SuccessPrompt.tsx
  2:35  warning  'View' is defined but never used    @typescript-eslint/no-unused-vars
  3:20  warning  'Easing' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/SocialsCard.tsx
    7:10  warning  'idToPeople' is defined but never used            @typescript-eslint/no-unused-vars
    9:10  warning  'SocialEventInfo' is defined but never used       @typescript-eslint/no-unused-vars
   54:7   warning  'hostProfile' is assigned a value but never used  @typescript-eslint/no-unused-vars
  529:10  warning  'alert' is defined but never used                 @typescript-eslint/no-unused-vars
  529:16  warning  'arg0' is defined but never used                  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/SocialsPage/Attending/EventCard.tsx
  1:24  warning  'updateInfo' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Components/SocialsPage/Hosting/DraftCard.tsx
    1:24  warning  'updateInfo' is defined but never used      @typescript-eslint/no-unused-vars
   38:9   warning  'Photo' is assigned a value but never used  @typescript-eslint/no-unused-vars
  167:16  warning  'hour' is defined but never used            @typescript-eslint/no-unused-vars
  168:18  warning  'minute' is defined but never used          @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/BioModal.tsx
  1:27  warning  'useEffect' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/BookVenueForm/BookVenueForm.tsx
   11:3   warning  'Modal' is defined but never used                                      @typescript-eslint/no-unused-vars
   17:10  warning  'BlurView' is defined but never used                                   @typescript-eslint/no-unused-vars
   20:10  warning  'Config' is defined but never used                                     @typescript-eslint/no-unused-vars
   36:10  warning  'toHosting' is defined but never used                                  @typescript-eslint/no-unused-vars
  158:16  warning  'editRes' is assigned a value but never used                           @typescript-eslint/no-unused-vars
  159:10  warning  'pushOneSignal' is assigned a value but never used                     @typescript-eslint/no-unused-vars
  159:25  warning  'oneSignalRes' is assigned a value but never used                      @typescript-eslint/no-unused-vars
  160:10  warning  'showLeavePageModal' is assigned a value but never used                @typescript-eslint/no-unused-vars
  160:30  warning  'setShowLeavePageModal' is assigned a value but never used             @typescript-eslint/no-unused-vars
  181:10  warning  'socialDraftModalVisible' is assigned a value but never used           @typescript-eslint/no-unused-vars
  184:35  warning  'isLoading' is assigned a value but never used                         @typescript-eslint/no-unused-vars
  184:46  warning  'error' is assigned a value but never used                             @typescript-eslint/no-unused-vars
  251:9   warning  'changeSocialDraftModalVisibility' is assigned a value but never used  @typescript-eslint/no-unused-vars
  267:9   warning  'toVenue' is assigned a value but never used                           @typescript-eslint/no-unused-vars
  278:9   warning  'authToken' is assigned a value but never used                         @typescript-eslint/no-unused-vars
  283:21  warning  'pushRes' is assigned a value but never used                           @typescript-eslint/no-unused-vars
  284:10  warning  'newEventId' is assigned a value but never used                        @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/BookVenueForm/CancelSocialForm.tsx
    1:10  warning  'Fonts' is defined but never used                       @typescript-eslint/no-unused-vars
  115:33  warning  'Common' is assigned a value but never used             @typescript-eslint/no-unused-vars
  115:41  warning  'SocialEventStyles' is assigned a value but never used  @typescript-eslint/no-unused-vars
  115:60  warning  'Colors' is assigned a value but never used             @typescript-eslint/no-unused-vars
  119:10  warning  'cancel' is assigned a value but never used             @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/BookVenueForm/SocialDraftModal.tsx
  4:10  warning  'navigate' is defined but never used         @typescript-eslint/no-unused-vars
  4:20  warning  'navigateForward' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/HomeContainer.tsx
  22:31  warning  'useUpdateUserDataMutation' is defined but never used  @typescript-eslint/no-unused-vars
  26:10  warning  'useLazyPushNotifQuery' is defined but never used      @typescript-eslint/no-unused-vars
  45:10  warning  'cityUpdated' is assigned a value but never used       @typescript-eslint/no-unused-vars
  52:10  warning  'notifs' is assigned a value but never used            @typescript-eslint/no-unused-vars
  52:18  warning  'setNotifs' is assigned a value but never used         @typescript-eslint/no-unused-vars
  58:21  warning  'pushRes' is assigned a value but never used           @typescript-eslint/no-unused-vars
  66:9   warning  'toDiscussions' is assigned a value but never used     @typescript-eslint/no-unused-vars
  69:9   warning  'accessToken' is assigned a value but never used       @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/InterestsModal.tsx
  13:10  warning  'Config' is defined but never used                 @typescript-eslint/no-unused-vars
  28:31  warning  'interestsRes' is assigned a value but never used  @typescript-eslint/no-unused-vars
  78:9   warning  'authToken' is assigned a value but never used     @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/CheckEmailPage.tsx
  13:22  warning  'setFormStatus' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/Components/Footer.tsx
  28:11  warning  'height' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/Components/InputBox.tsx
   2:10  warning  'Text' is defined but never used            @typescript-eslint/no-unused-vars
   2:27  warning  'TextStyle' is defined but never used       @typescript-eslint/no-unused-vars
   6:8   warning  'PropTypes' is defined but never used       @typescript-eslint/no-unused-vars
  34:11  warning  'Fonts' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/EmailPage.tsx
   1:28  warning  'useRef' is defined but never used             @typescript-eslint/no-unused-vars
   2:22  warning  'Text' is defined but never used               @typescript-eslint/no-unused-vars
  21:20  warning  'emailRes' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/LoginPage.tsx
   2:17  warning  'isValidElement' is defined but never used            @typescript-eslint/no-unused-vars
   8:10  warning  'onLogin' is defined but never used                   @typescript-eslint/no-unused-vars
   8:19  warning  'NavigationControllerData' is defined but never used  @typescript-eslint/no-unused-vars
   8:45  warning  'onCheckEmail' is defined but never used              @typescript-eslint/no-unused-vars
  18:8   warning  'OneSignal' is defined but never used                 @typescript-eslint/no-unused-vars
  47:22  warning  'setToastError' is assigned a value but never used    @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/LoginFlow/SignUpPage.tsx
   2:38  warning  'createContext' is defined but never used             @typescript-eslint/no-unused-vars
  11:10  warning  'NavigationControllerData' is defined but never used  @typescript-eslint/no-unused-vars
  11:36  warning  'onSignUp' is defined but never used                  @typescript-eslint/no-unused-vars
  53:22  warning  'setToastError' is assigned a value but never used    @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/Onboarding/LocationPage.tsx
   2:41  warning  'SafeAreaView' is defined but never used            @typescript-eslint/no-unused-vars
   4:20  warning  'Params' is defined but never used                  @typescript-eslint/no-unused-vars
   4:28  warning  'postData' is defined but never used                @typescript-eslint/no-unused-vars
   5:10  warning  'Config' is defined but never used                  @typescript-eslint/no-unused-vars
  15:10  warning  'locationQuery' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/Onboarding/OnboardingScreen.tsx
  2:36  warning  'Dimensions' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileContainers/EarnPointsPage.tsx
  13:10  warning  'Colors' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileIndustryTagsModal.tsx
  23:32  warning  'professionRes' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/AccountInfo.tsx
  5:16  warning  'Text' is defined but never used         @typescript-eslint/no-unused-vars
  6:10  warning  'useSelector' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ChangePassword.tsx
  1:25  warning  'InputBar' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ChooseLocation.tsx
  28:9  warning  'editCity' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ConnectInstagram.tsx
   3:10  warning  'navigateForward' is defined but never used   @typescript-eslint/no-unused-vars
  27:16  warning  'setName' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ConnectLinkedIn.tsx
  1:45  warning  'InputField' is defined but never used       @typescript-eslint/no-unused-vars
  3:10  warning  'navigateForward' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ConnectSocials.tsx
  4:16  warning  'Text' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ConnectTwitter.tsx
  1:45  warning  'InputBar' is defined but never used         @typescript-eslint/no-unused-vars
  3:10  warning  'navigateForward' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/PrivacySafety.tsx
  4:16  warning  'Text' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ProfileForgotPassword.tsx
  1:25  warning  'InputBar' is defined but never used         @typescript-eslint/no-unused-vars
  3:10  warning  'navigateForward' is defined but never used  @typescript-eslint/no-unused-vars
  7:52  warning  'Button' is defined but never used           @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/ProfileSettings.tsx
   6:10  warning  'navigateAndSimpleReset' is defined but never used  @typescript-eslint/no-unused-vars
  10:10  warning  'navigateAndReset' is defined but never used        @typescript-eslint/no-unused-vars
  16:9   warning  'styles' is assigned a value but never used         @typescript-eslint/no-unused-vars
  26:10  warning  'backdrop' is assigned a value but never used       @typescript-eslint/no-unused-vars
  26:20  warning  'setBackdrop' is assigned a value but never used    @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/ProfileSettings/SMSNotifications.tsx
  3:10  warning  'navigateForward' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/SampleSocialDetail/SampleDirectMessagePage.tsx
   2:35  warning  'useEffect' is defined but never used              @typescript-eslint/no-unused-vars
   4:10  warning  'useSelector' is defined but never used            @typescript-eslint/no-unused-vars
  11:10  warning  'Config' is defined but never used                 @typescript-eslint/no-unused-vars
  22:11  warning  'refDoubleButtonModal' is defined but never used   @typescript-eslint/no-unused-vars
  32:23  warning  'postRes' is assigned a value but never used       @typescript-eslint/no-unused-vars
  33:20  warning  'inSocRes' is assigned a value but never used      @typescript-eslint/no-unused-vars
  34:16  warning  'pushRes' is assigned a value but never used       @typescript-eslint/no-unused-vars
  35:25  warning  'oneSignalRes' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/SampleSocialDetail/SampleSocialDetailPage.tsx
   6:3   warning  'SocialEventInfo' is defined but never used          @typescript-eslint/no-unused-vars
  16:10  warning  'useGetUserDataQuery' is defined but never used      @typescript-eslint/no-unused-vars
  30:10  warning  'ProfileContext' is defined but never used           @typescript-eslint/no-unused-vars
  55:20  warning  'inSocRes' is assigned a value but never used        @typescript-eslint/no-unused-vars
  56:22  warning  'inWlRes' is assigned a value but never used         @typescript-eslint/no-unused-vars
  57:21  warning  'outSocRes' is assigned a value but never used       @typescript-eslint/no-unused-vars
  58:23  warning  'outWlRes' is assigned a value but never used        @typescript-eslint/no-unused-vars
  59:21  warning  'pushRes' is assigned a value but never used         @typescript-eslint/no-unused-vars
  60:25  warning  'oneSignalRes' is assigned a value but never used    @typescript-eslint/no-unused-vars
  80:10  warning  'isPublished' is assigned a value but never used     @typescript-eslint/no-unused-vars
  80:23  warning  'setIsPublished' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Containers/SampleVenueDetail/SampleVenueDetailPage.tsx
  88:11  warning  'Layout' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Navigators/Main.tsx
  10:10  warning  'BlurView' is defined but never used                @typescript-eslint/no-unused-vars
  25:22  warning  'setNewMessage' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/Context/ProfileContext.tsx
  5:20  warning  's' is defined but never used  @typescript-eslint/no-unused-vars
  9:17  warning  's' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/Context/SocialsContext.tsx
  5:14  warning  's' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/Context/VenusesContext.tsx
  5:14  warning  's' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/Notifications.tsx
   1:17  warning  'useState' is defined but never used              @typescript-eslint/no-unused-vars
   1:27  warning  'useEffect' is defined but never used             @typescript-eslint/no-unused-vars
   8:3   warning  'SafeAreaView' is defined but never used          @typescript-eslint/no-unused-vars
  13:7   warning  'Tab' is assigned a value but never used          @typescript-eslint/no-unused-vars
  14:8   warning  'System' is defined but never used                @typescript-eslint/no-unused-vars
  15:8   warning  'Messages' is defined but never used              @typescript-eslint/no-unused-vars
  16:8   warning  'SocialsNotifications' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/NotificationsTab/Messages.tsx
   3:10  warning  'MessageNotification' is defined but never used     @typescript-eslint/no-unused-vars
   9:19  warning  'b' is defined but never used                       @typescript-eslint/no-unused-vars
  20:11  warning  'Layout' is assigned a value but never used         @typescript-eslint/no-unused-vars
  21:9   warning  'scrollViewRef' is assigned a value but never used  @typescript-eslint/no-unused-vars
  22:22  warning  'setToastError' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/NotificationsTab/SocialsNotifications.tsx
   1:10  warning  'SocialNotification' is defined but never used      @typescript-eslint/no-unused-vars
   8:19  warning  'b' is defined but never used                       @typescript-eslint/no-unused-vars
  19:22  warning  'setToastError' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/NotificationsTab/System.tsx
   3:10  warning  'SystemNotification' is defined but never used      @typescript-eslint/no-unused-vars
   8:19  warning  'b' is defined but never used                       @typescript-eslint/no-unused-vars
  19:22  warning  'setToastError' is assigned a value but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/SocialsTab/Attending.tsx
  145:24  warning  'item' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Screens/SocialsTab/Hosting.tsx
  144:24  warning  'item' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Services/modules/venues/endpoints.ts
  2:10  warning  'Venue' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Store/Actions.ts
  2:10  warning  'string' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Theme/components/NotificationStyles.ts
  1:8  warning  'React' is defined but never used  @typescript-eslint/no-unused-vars

/Users/ayman/Desktop/Ayman/mobile_app/amelia-mobile/src/Util/Linkedin/Authorization.tsx
   30:11  warning  'test' is assigned a value but never used        @typescript-eslint/no-unused-vars
  150:9   warning  'testToken2' is assigned a value but never used  @typescript-eslint/no-unused-vars

✖ 181 problems (0 errors, 181 warnings)

✨  Done in 6.77s.
```

## Channel Overriding for Indoor Drone Testing
(Guide on how to override the RC for indoor control of drone - which is unstable but possible)[https://dronekit-python.readthedocs.io/en/latest/examples/channel_overrides.html#example-channel-overrides]

## Terminal Log After First 3 Tests
```typescript
pi@raspberrypi:~/droneTest $ python real_test_1.py ############# FIRST TEST #############
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
Taking off!
('Altitude:', -0.066)
('Altitude:', -0.056)
('Altitude:', 0.105)
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.407)
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:Radio Failsafe
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', 0.798)
('Altitude:', 0.748)
('Altitude:', 0.864)
('Altitude:', 1.122)
Reached target altitude
Take off complete
Now let's land
pi@raspberrypi:~/droneTest $ python real_test_1.py 
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
Taking off!
('Altitude:', 0.109)
('Altitude:', 0.187)
('Altitude:', 0.222)
('Altitude:', 0.528)
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.893)
('Altitude:', 0.95)
Reached target altitude
Take off complete
Now let's land
pi@raspberrypi:~/droneTest $ python real_test_2.py ############# SECOND TEST #############
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
Taking off!
('Altitude:', 0.024)
Latitude: 28.382081, Longitude: 36.482996
('Altitude:', 0.028)
Latitude: 28.382080, Longitude: 36.482996
('Altitude:', 0.076)
Latitude: 28.382080, Longitude: 36.482996
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.659)
Latitude: 28.382080, Longitude: 36.482996
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 1.55)
Latitude: 28.382080, Longitude: 36.482996
('Altitude:', 1.834)
Latitude: 28.382081, Longitude: 36.482996
('Altitude:', 1.927)
Latitude: 28.382080, Longitude: 36.482996
Reached target altitude
Take off complete
Now let's land
pi@raspberrypi:~/droneTest $ python real_test_3.py ############# THIRD TEST #############
WARNING:dronekit:Link timeout, no heartbeat in last 5 seconds
ERROR:dronekit.mavlink:Exception in MAVLink input loop
Traceback (most recent call last):
  File "/usr/local/lib/python2.7/dist-packages/dronekit/mavlink.py", line 211, in mavlink_thread_in
    fn(self)
  File "/usr/local/lib/python2.7/dist-packages/dronekit/__init__.py", line 1371, in listener
    self._heartbeat_error)
APIException: No heartbeat in 30 seconds, aborting.
Traceback (most recent call last):
  File "real_test_3.py", line 5, in <module>
    vehicle = connect('/dev/ttyAMA0', wait_ready=True)  # /dev/ttyS0
  File "/usr/local/lib/python2.7/dist-packages/dronekit/__init__.py", line 3166, in connect
    vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)
  File "/usr/local/lib/python2.7/dist-packages/dronekit/__init__.py", line 2275, in initialize
    raise APIException('Timeout in initializing connection.')
dronekit.APIException: Timeout in initializing connection.
pi@raspberrypi:~/droneTest $ python real_test_3.py ############# THIRD TEST #############
WARNING:dronekit:Link timeout, no heartbeat in last 5 seconds
^Z
[1]+  Stopped                 python real_test_3.py
pi@raspberrypi:~/droneTest $ python real_test_2.py
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
Taking off!
('Altitude:', -0.062)
Latitude: 28.382075, Longitude: 36.482991
('Altitude:', 0.026)
Latitude: 28.382075, Longitude: 36.482991
('Altitude:', 0.142)
Latitude: 28.382075, Longitude: 36.482991
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.504)
Latitude: 28.382075, Longitude: 36.482991
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 1.562)
Latitude: 28.382071, Longitude: 36.482990
('Altitude:', 1.916)
Latitude: 28.382071, Longitude: 36.482990
Reached target altitude
Take off complete
Now let's land
pi@raspberrypi:~/droneTest $ python real_test_4.py ############# FOURTH TEST #############
python: can't open file 'real_test_4.py': [Errno 2] No such file or directory
pi@raspberrypi:~/droneTest $ python real_test4.py
  File "real_test4.py", line 66
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
                                                                        ^
IndentationError: unindent does not match any outer indentation level
pi@raspberrypi:~/droneTest $ python real_test4.py
  File "real_test4.py", line 66
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                                                                       ^
IndentationError: unindent does not match any outer indentation level
pi@raspberrypi:~/droneTest $ python real_test_2.py ############# SECOND TEST AGAIN (With different height)#############
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
Taking off!
('Altitude:', -0.009)
Latitude: 28.382078, Longitude: 36.482986
('Altitude:', -0.043)
Latitude: 28.382078, Longitude: 36.482986
('Altitude:', -0.141)
Latitude: 28.382078, Longitude: 36.482986
('Altitude:', 0.313)
Latitude: 28.382078, Longitude: 36.482986
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 2.178)
Latitude: 28.382078, Longitude: 36.482985
('Altitude:', 3.423)
Latitude: 28.382078, Longitude: 36.482987
('Altitude:', 3.999)
Latitude: 28.382078, Longitude: 36.482987
Reached target altitude
Take off complete
Now let's land

```

## RP4 Wi-fi Issues
If the access point is no longer showing up on your Raspberry Pi 4, there could be several reasons behind this issue. Here are some troubleshooting steps you can try:

1. **Check Power and Hardware Connections:**
   - Make sure your Raspberry Pi is receiving power and all the necessary cables (such as the power adapter, HDMI, Ethernet, etc.) are properly connected.
   - Ensure that the Wi-Fi adapter or module is securely connected to the Raspberry Pi.

2. **Restart the Raspberry Pi:**
   - Sometimes a simple restart can resolve connectivity issues. Reboot your Raspberry Pi and see if the access point appears again.

3. **Verify Wi-Fi Configuration:**
   - Access the Raspberry Pi's terminal or SSH into it and check the Wi-Fi configuration files.
   - Run the following command to open the configuration file in a text editor:
     ```
     sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
     ```
   - Ensure that the access point details (SSID and password) are correctly entered in the configuration file.

4. **Check Wi-Fi Adapter Compatibility:**
   - Confirm that your Wi-Fi adapter is compatible with the Raspberry Pi 4 and supported by the Raspberry Pi OS.
   - Look for any specific installation steps or drivers required for your Wi-Fi adapter.

5. **Update and Upgrade Packages:**
   - Run the following commands to update and upgrade the Raspberry Pi's packages:
     ```
     sudo apt update
     sudo apt upgrade
     ```
   - This ensures that you have the latest software and firmware installed, which can potentially fix any compatibility issues.

6. **Check Wi-Fi Adapter Power Management:**
   - Some Wi-Fi adapters have power-saving features that can cause connectivity problems. Disable power management for the Wi-Fi adapter by creating a configuration file.
   - Run the following command to create the configuration file:
     ```
     sudo nano /etc/modprobe.d/8192cu.conf
     ```
   - Add the following line to the file and save it:
     ```
     options 8192cu rtw_power_mgnt=0 rtw_enusbss=0
     ```
   - Reboot the Raspberry Pi and check if the access point appears.

7. **Verify Access Point Settings:**
   - If you are setting up an access point on the Raspberry Pi, double-check the access point configuration files to ensure they are correctly configured.
   - Confirm that the necessary software (such as `hostapd` and `dnsmasq`) is properly installed and configured.

8. **Check for Interference or Range Issues:**
   - If you are using a Wi-Fi dongle, try moving the Raspberry Pi closer to the access point to rule out any range or interference problems.
   - Test the access point with another device to ensure it is working correctly.

If none of the above steps resolve the issue, it's possible that there may be a hardware problem with your Wi-Fi adapter or Raspberry Pi. In such cases, consider trying a different Wi-Fi adapter or contacting technical support for further assistance.

## Logs from Testing - Part 4
```typescript
pi@raspberrypi:~/droneTest $ python real_test_5.py
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 16 to 15
Taking off!
('Altitude:', 0.037)
Latitude: 28.382083, Longitude: 36.483037
Roll: -0.01, Pitch: 0.01, Yaw: 2.37
('Altitude:', 0.015)
Latitude: 28.382082, Longitude: 36.483037
Roll: -0.01, Pitch: 0.01, Yaw: 2.37
('Altitude:', 0.019)
Latitude: 28.382082, Longitude: 36.483037
Roll: -0.01, Pitch: 0.00, Yaw: 2.37
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.65)
Latitude: 28.382081, Longitude: 36.483038
Roll: -0.02, Pitch: 0.02, Yaw: 2.46
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 2.552)
Latitude: 28.382080, Longitude: 36.483038
Roll: 0.02, Pitch: 0.04, Yaw: 2.96
('Altitude:', 3.952)
Latitude: 28.382081, Longitude: 36.483038
Roll: 0.01, Pitch: 0.03, Yaw: 2.79
('Altitude:', 4.746)
Latitude: 28.382081, Longitude: 36.483037
Roll: -0.01, Pitch: 0.04, Yaw: 2.84
('Altitude:', 4.929)
Latitude: 28.382082, Longitude: 36.483036
Roll: -0.02, Pitch: 0.02, Yaw: 2.87
Reached target altitude
Take off complete
Starting to land! Prepare for landing!
pi@raspberrypi:~/droneTest $ ls
First_Test.py   real_test_2.py  real_test_5.py       real_test_p2p_1.py
no_gps_test.py  real_test_3.py  real_test_NEWS_1.py  real_test_p2p_2.py
real_test_1.py  real_test_4.py  real_test_NEWS_2.py
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 17 to 15
Taking off!
('Altitude:', -0.045)
Latitude: 28.382074, Longitude: 36.483030
Roll: -0.00, Pitch: 0.05, Yaw: 2.50
('Altitude:', -0.112)
Latitude: 28.382073, Longitude: 36.483030
Roll: -0.00, Pitch: 0.05, Yaw: 2.50
('Altitude:', -0.17)
Latitude: 28.382073, Longitude: 36.483030
Roll: -0.00, Pitch: 0.04, Yaw: 2.50
WARNING:autopilot:Radio Failsafe
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', 0.028)
Latitude: 28.382072, Longitude: 36.483031
Roll: -0.01, Pitch: 0.03, Yaw: 2.56
('Altitude:', 0.249)
Latitude: 28.382072, Longitude: 36.483032
Roll: -0.01, Pitch: 0.05, Yaw: 2.59
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.46)
Latitude: 28.382072, Longitude: 36.483032
Roll: 0.02, Pitch: 0.05, Yaw: 2.56
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.573)
Latitude: 28.382071, Longitude: 36.483031
Roll: 0.03, Pitch: 0.03, Yaw: 2.94
('Altitude:', 0.22)
Latitude: 28.382072, Longitude: 36.483030
Roll: 0.01, Pitch: 0.05, Yaw: 2.94
WARNING:autopilot:Radio Failsafe
('Altitude:', 0.169)
Latitude: 28.382072, Longitude: 36.483029
Roll: -0.01, Pitch: 0.03, Yaw: 2.87
('Altitude:', 0.133)
Latitude: 28.382072, Longitude: 36.483029
Roll: -0.01, Pitch: 0.02, Yaw: 2.81
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', 0.159)
Latitude: 28.382072, Longitude: 36.483029
Roll: -0.01, Pitch: 0.00, Yaw: 2.77
WARNING:autopilot:Radio Failsafe - Disarming
('Altitude:', 0.176)
Latitude: 28.382072, Longitude: 36.483029
Roll: -0.01, Pitch: -0.00, Yaw: 2.74
('Altitude:', 0.161)
Latitude: 28.382072, Longitude: 36.483029
Roll: -0.01, Pitch: -0.00, Yaw: 2.71
('Altitude:', 0.106)
Latitude: 28.382073, Longitude: 36.483029
Roll: -0.01, Pitch: -0.00, Yaw: 2.70
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', 0.061)
Latitude: 28.382073, Longitude: 36.483028
Roll: -0.01, Pitch: -0.00, Yaw: 2.68
('Altitude:', 0.038)
Latitude: 28.382074, Longitude: 36.483027
Roll: -0.01, Pitch: -0.01, Yaw: 2.67
('Altitude:', 0.009)
Latitude: 28.382075, Longitude: 36.483027
Roll: -0.01, Pitch: -0.01, Yaw: 2.67
('Altitude:', 0.006)
Latitude: 28.382076, Longitude: 36.483026
Roll: -0.01, Pitch: -0.01, Yaw: 2.66
WARNING:autopilot:Radio Failsafe - Disarming
('Altitude:', 0.011)
Latitude: 28.382077, Longitude: 36.483025
Roll: -0.01, Pitch: -0.01, Yaw: 2.66
^Z
[2]+  Stopped                 python real_test_p2p_1.py
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
^Z
[3]+  Stopped                 python real_test_p2p_1.py
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 17 to 15
Taking off!
('Altitude:', -0.035)
Latitude: 28.382103, Longitude: 36.483000
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', -0.126)
Latitude: 28.382103, Longitude: 36.482999
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', -0.151)
Latitude: 28.382103, Longitude: 36.482999
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', -0.029)
Latitude: 28.382103, Longitude: 36.482999
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', 0.025)
Latitude: 28.382103, Longitude: 36.482999
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', 0.068)
Latitude: 28.382103, Longitude: 36.482999
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', 0.056)
Latitude: 28.382103, Longitude: 36.482998
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', 0.073)
Latitude: 28.382103, Longitude: 36.482998
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
('Altitude:', 0.083)
Latitude: 28.382102, Longitude: 36.482997
Roll: 0.02, Pitch: -0.01, Yaw: 2.48
^Z
[4]+  Stopped                 python real_test_p2p_1.py
pi@raspberrypi:~/droneTest $ python real_test_5.py
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
Basic pre-arm checks
Starting .....
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 17 to 15
Taking off!
('Altitude:', -0.011)
Latitude: 28.382099, Longitude: 36.483003
Roll: -0.01, Pitch: -0.00, Yaw: 2.41
('Altitude:', -0.058)
Latitude: 28.382099, Longitude: 36.483003
Roll: -0.01, Pitch: -0.00, Yaw: 2.41
('Altitude:', -0.083)
Latitude: 28.382099, Longitude: 36.483004
Roll: -0.01, Pitch: -0.02, Yaw: 2.40
('Altitude:', 0.187)
Latitude: 28.382098, Longitude: 36.483004
Roll: 0.01, Pitch: 0.00, Yaw: 2.49
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 2.157)
Latitude: 28.382096, Longitude: 36.483003
Roll: -0.08, Pitch: 0.08, Yaw: 3.00
('Altitude:', 3.878)
Latitude: 28.382095, Longitude: 36.483002
Roll: -0.04, Pitch: 0.09, Yaw: 2.81
('Altitude:', 4.727)
Latitude: 28.382096, Longitude: 36.483000
Roll: -0.13, Pitch: 0.06, Yaw: 2.53
('Altitude:', 5.188)
Latitude: 28.382099, Longitude: 36.483002
Roll: -0.07, Pitch: 0.03, Yaw: 2.97
Reached target altitude
Take off complete
Starting to land! Prepare for landing!
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 18 to 15
Taking off!
('Altitude:', 0.054)
Latitude: 28.382104, Longitude: 36.483016
Roll: -0.06, Pitch: 0.01, Yaw: 2.65
('Altitude:', 0.102)
Latitude: 28.382104, Longitude: 36.483015
Roll: -0.06, Pitch: 0.01, Yaw: 2.66
('Altitude:', 0.16)
Latitude: 28.382104, Longitude: 36.483016
Roll: -0.05, Pitch: 0.02, Yaw: 2.68
('Altitude:', 0.427)
Latitude: 28.382101, Longitude: 36.483015
Roll: 0.97, Pitch: -0.55, Yaw: 2.34
('Altitude:', 0.087)
Latitude: 28.382098, Longitude: 36.483015
Roll: 2.80, Pitch: 0.26, Yaw: 1.94
('Altitude:', 0.283)
Latitude: 28.382099, Longitude: 36.483019
Roll: 2.63, Pitch: 0.10, Yaw: 2.14
CRITICAL:autopilot:Crash: Disarming: AngErr=132>30, Accel=0.4<3.0
('Altitude:', 0.287)
Latitude: 28.382102, Longitude: 36.483021
Roll: 2.65, Pitch: 0.20, Yaw: 2.88
('Altitude:', -0.498)
Latitude: 28.382101, Longitude: 36.483022
Roll: 2.66, Pitch: 0.09, Yaw: 2.74
('Altitude:', -0.915)
Latitude: 28.382100, Longitude: 36.483022
Roll: 2.78, Pitch: 0.09, Yaw: 2.66
('Altitude:', -1.117)
Latitude: 28.382099, Longitude: 36.483022
Roll: 2.78, Pitch: 0.11, Yaw: 2.70
('Altitude:', -1.242)
Latitude: 28.382099, Longitude: 36.483022
Roll: 2.78, Pitch: 0.11, Yaw: 2.70
('Altitude:', -0.975)
Latitude: 28.382099, Longitude: 36.483020
Roll: 2.31, Pitch: 0.39, Yaw: 3.01
('Altitude:', -0.807)
Latitude: 28.382099, Longitude: 36.483021
Roll: 1.47, Pitch: 0.28, Yaw: 1.62
('Altitude:', -0.397)
Latitude: 28.382101, Longitude: 36.483019
Roll: 0.88, Pitch: -0.09, Yaw: 1.32
('Altitude:', -0.17)
Latitude: 28.382102, Longitude: 36.483018
Roll: 0.62, Pitch: -0.08, Yaw: 1.35
^Z
[5]+  Stopped                 python real_test_p2p_1.py
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
WARNING:autopilot:Radio Failsafe - Disarming
CRITICAL:autopilot:PreArm: Throttle below failsafe
CRITICAL:autopilot:PreArm: Radio failsafe on
WARNING:autopilot:Radio Failsafe Cleared
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 18 to 15
Taking off!
('Altitude:', 0.007)
Latitude: 28.382101, Longitude: 36.483023
Roll: -0.07, Pitch: -0.04, Yaw: 2.29
('Altitude:', 0.029)
Latitude: 28.382101, Longitude: 36.483023
Roll: -0.07, Pitch: -0.04, Yaw: 2.29
('Altitude:', 0.063)
Latitude: 28.382101, Longitude: 36.483024
Roll: -0.07, Pitch: -0.04, Yaw: 2.29
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.423)
Latitude: 28.382100, Longitude: 36.483025
Roll: -0.03, Pitch: 0.00, Yaw: 2.41
('Altitude:', 1.844)
Latitude: 28.382100, Longitude: 36.483025
Roll: -0.01, Pitch: -0.00, Yaw: 2.96
('Altitude:', 2.772)
Latitude: 28.382100, Longitude: 36.483024
Roll: -0.02, Pitch: 0.02, Yaw: 2.79
('Altitude:', 2.911)
Latitude: 28.382100, Longitude: 36.483023
Roll: -0.02, Pitch: 0.01, Yaw: 2.98
('Altitude:', 2.874)
Latitude: 28.382102, Longitude: 36.483023
Roll: -0.01, Pitch: -0.01, Yaw: 2.90
('Altitude:', 2.846)
Latitude: 28.382102, Longitude: 36.483024
Roll: -0.01, Pitch: -0.01, Yaw: 2.96
('Altitude:', 2.841)
Latitude: 28.382102, Longitude: 36.483024
Roll: -0.02, Pitch: 0.01, Yaw: 2.92
('Altitude:', 2.899)
Latitude: 28.382101, Longitude: 36.483024
Roll: -0.02, Pitch: 0.01, Yaw: 2.92
('Altitude:', 2.878)
Latitude: 28.382100, Longitude: 36.483024
Roll: -0.02, Pitch: 0.01, Yaw: 2.90
('Altitude:', 2.873)
Latitude: 28.382100, Longitude: 36.483024
Roll: -0.03, Pitch: 0.01, Yaw: 2.89
('Altitude:', 2.894)
Latitude: 28.382100, Longitude: 36.483023
Roll: -0.01, Pitch: -0.01, Yaw: 2.86
('Altitude:', 2.875)
Latitude: 28.382100, Longitude: 36.483023
Roll: -0.04, Pitch: 0.02, Yaw: 2.84
('Altitude:', 2.858)
Latitude: 28.382099, Longitude: 36.483023
Roll: -0.03, Pitch: 0.02, Yaw: 2.84
CRITICAL:autopilot:EKF3 lane switch 1
WARNING:autopilot:EKF primary changed:1
('Altitude:', -0.147)
Latitude: 28.382099, Longitude: 36.483024
Roll: -0.17, Pitch: -0.03, Yaw: 2.75
CRITICAL:autopilot:Vibration compensation ON
CRITICAL:autopilot:EKF variance
CRITICAL:autopilot:EKF Failsafe: changed to LAND Mode
('Altitude:', -5.661)
Latitude: 28.382098, Longitude: 36.483018
Roll: -0.05, Pitch: -0.07, Yaw: 2.66
CRITICAL:autopilot:GPS Glitch or Compass error
('Altitude:', -6.406)
Latitude: 28.382096, Longitude: 36.483018
Roll: -0.06, Pitch: -0.07, Yaw: 2.62
CRITICAL:autopilot:EKF Failsafe Cleared
('Altitude:', 0.608)
Latitude: 28.382098, Longitude: 36.483021
Roll: -0.07, Pitch: -0.07, Yaw: 2.60
('Altitude:', 0.482)
Latitude: 28.382100, Longitude: 36.483021
Roll: -0.07, Pitch: -0.07, Yaw: 2.59
('Altitude:', 0.247)
Latitude: 28.382100, Longitude: 36.483022
Roll: -0.08, Pitch: -0.06, Yaw: 2.60
('Altitude:', 0.083)
Latitude: 28.382100, Longitude: 36.483021
Roll: -0.09, Pitch: -0.06, Yaw: 2.60
WARNING:autopilot:EKF primary changed:0
('Altitude:', 0.205)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.05, Pitch: -0.07, Yaw: 2.60
('Altitude:', 0.181)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.05, Pitch: -0.06, Yaw: 2.61
('Altitude:', 0.189)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.05, Pitch: -0.06, Yaw: 2.62
('Altitude:', 0.044)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.05, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.071)
Latitude: 28.382099, Longitude: 36.483021
Roll: -0.05, Pitch: -0.06, Yaw: 2.63
CRITICAL:autopilot:Glitch cleared
('Altitude:', 0.156)
Latitude: 28.382099, Longitude: 36.483021
Roll: -0.05, Pitch: -0.05, Yaw: 2.63
('Altitude:', 0.244)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.05, Pitch: -0.05, Yaw: 2.63
('Altitude:', 0.235)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.04, Pitch: -0.04, Yaw: 2.63
('Altitude:', 0.118)
Latitude: 28.382099, Longitude: 36.483022
Roll: -0.04, Pitch: -0.03, Yaw: 2.63
('Altitude:', 0.046)
Latitude: 28.382099, Longitude: 36.483021
Roll: -0.03, Pitch: -0.03, Yaw: 2.63
('Altitude:', -0.006)
Latitude: 28.382099, Longitude: 36.483021
Roll: -0.03, Pitch: -0.02, Yaw: 2.63
CRITICAL:autopilot:Vibration compensation OFF
('Altitude:', 0.02)
Latitude: 28.382099, Longitude: 36.483021
Roll: -0.03, Pitch: -0.02, Yaw: 2.63
('Altitude:', 0.097)
Latitude: 28.382100, Longitude: 36.483021
Roll: -0.04, Pitch: -0.01, Yaw: 2.63
('Altitude:', 0.182)
Latitude: 28.382100, Longitude: 36.483021
Roll: -0.04, Pitch: -0.02, Yaw: 2.62
('Altitude:', 0.188)
Latitude: 28.382100, Longitude: 36.483020
Roll: -0.04, Pitch: -0.02, Yaw: 2.62
('Altitude:', 0.146)
Latitude: 28.382100, Longitude: 36.483019
Roll: -0.04, Pitch: -0.02, Yaw: 2.62
('Altitude:', 0.13)
Latitude: 28.382100, Longitude: 36.483019
Roll: -0.04, Pitch: -0.02, Yaw: 2.62
('Altitude:', 0.202)
Latitude: 28.382100, Longitude: 36.483019
Roll: -0.04, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.229)
Latitude: 28.382101, Longitude: 36.483018
Roll: -0.04, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.174)
Latitude: 28.382102, Longitude: 36.483018
Roll: -0.04, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.101)
Latitude: 28.382101, Longitude: 36.483017
Roll: -0.04, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.105)
Latitude: 28.382101, Longitude: 36.483017
Roll: -0.03, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.182)
Latitude: 28.382101, Longitude: 36.483016
Roll: -0.04, Pitch: -0.02, Yaw: 2.61
('Altitude:', 0.166)
Latitude: 28.382101, Longitude: 36.483015
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.166)
Latitude: 28.382101, Longitude: 36.483014
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.175)
Latitude: 28.382101, Longitude: 36.483014
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.163)
Latitude: 28.382101, Longitude: 36.483013
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.172)
Latitude: 28.382101, Longitude: 36.483013
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.185)
Latitude: 28.382101, Longitude: 36.483012
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.239)
Latitude: 28.382101, Longitude: 36.483012
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.231)
Latitude: 28.382101, Longitude: 36.483011
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.212)
Latitude: 28.382101, Longitude: 36.483011
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.191)
Latitude: 28.382100, Longitude: 36.483011
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.21)
Latitude: 28.382100, Longitude: 36.483011
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.228)
Latitude: 28.382100, Longitude: 36.483011
Roll: -0.03, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.274)
Latitude: 28.382100, Longitude: 36.483012
Roll: -0.04, Pitch: -0.02, Yaw: 2.60
('Altitude:', 0.355)
Latitude: 28.382100, Longitude: 36.483013
Roll: -0.21, Pitch: -0.70, Yaw: 2.82
('Altitude:', 0.167)
Latitude: 28.382100, Longitude: 36.483012
Roll: -0.10, Pitch: -0.73, Yaw: 2.75
('Altitude:', -0.058)
Latitude: 28.382101, Longitude: 36.483011
Roll: -0.00, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.146)
Latitude: 28.382101, Longitude: 36.483012
Roll: -0.00, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.237)
Latitude: 28.382101, Longitude: 36.483013
Roll: -0.00, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.269)
Latitude: 28.382100, Longitude: 36.483013
Roll: -0.01, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.268)
Latitude: 28.382100, Longitude: 36.483015
Roll: -0.01, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.231)
Latitude: 28.382100, Longitude: 36.483015
Roll: -0.00, Pitch: -0.06, Yaw: 2.61
('Altitude:', -0.176)
Latitude: 28.382100, Longitude: 36.483016
Roll: -0.00, Pitch: -0.06, Yaw: 2.62
('Altitude:', -0.153)
Latitude: 28.382100, Longitude: 36.483016
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', -0.092)
Latitude: 28.382100, Longitude: 36.483017
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', -0.048)
Latitude: 28.382100, Longitude: 36.483017
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.008)
Latitude: 28.382100, Longitude: 36.483017
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
('Altitude:', 0.07)
Latitude: 28.382099, Longitude: 36.483018
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.123)
Latitude: 28.382099, Longitude: 36.483018
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.153)
Latitude: 28.382100, Longitude: 36.483018
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.158)
Latitude: 28.382100, Longitude: 36.483017
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.17)
Latitude: 28.382100, Longitude: 36.483016
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.197)
Latitude: 28.382100, Longitude: 36.483017
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.172)
Latitude: 28.382100, Longitude: 36.483016
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.144)
Latitude: 28.382100, Longitude: 36.483016
Roll: -0.00, Pitch: -0.05, Yaw: 2.62
('Altitude:', 0.116)
Latitude: 28.382101, Longitude: 36.483016
Roll: -0.01, Pitch: -0.06, Yaw: 2.62
('Altitude:', 0.298)
Latitude: 28.382101, Longitude: 36.483017
Roll: -0.01, Pitch: -0.06, Yaw: 2.62
('Altitude:', 0.241)
Latitude: 28.382100, Longitude: 36.483018
Roll: -0.01, Pitch: -0.06, Yaw: 2.62
('Altitude:', 0.225)
Latitude: 28.382100, Longitude: 36.483018
Roll: -0.01, Pitch: -0.06, Yaw: 2.62
('Altitude:', 0.209)
Latitude: 28.382100, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.175)
Latitude: 28.382100, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.195)
Latitude: 28.382101, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.183)
Latitude: 28.382101, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.174)
Latitude: 28.382101, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.199)
Latitude: 28.382101, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.166)
Latitude: 28.382101, Longitude: 36.483018
Roll: -0.00, Pitch: -0.06, Yaw: 2.63
('Altitude:', 0.169)
Latitude: 28.382102, Longitude: 36.483018
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
('Altitude:', 0.223)
Latitude: 28.382102, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
('Altitude:', 0.24)
Latitude: 28.382102, Longitude: 36.483020
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
('Altitude:', 0.283)
Latitude: 28.382102, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
('Altitude:', 0.249)
Latitude: 28.382102, Longitude: 36.483019
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
('Altitude:', 0.224)
Latitude: 28.382102, Longitude: 36.483020
Roll: -0.01, Pitch: -0.06, Yaw: 2.64
^Z
[6]+  Stopped                 python real_test_p2p_1.py
pi@raspberrypi:~/droneTest $ python real_test_p2p_1.py
Connecting to vehicle on: /dev/ttyAMA0
WARNING:autopilot:Radio Failsafe - Disarming
WARNING:autopilot:Radio Failsafe Cleared
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 18 to 15
Taking off!
('Altitude:', -0.019)
Latitude: 28.382107, Longitude: 36.483023
Roll: -0.02, Pitch: -0.05, Yaw: 2.68
('Altitude:', -0.05)
Latitude: 28.382108, Longitude: 36.483022
Roll: -0.02, Pitch: -0.05, Yaw: 2.68
('Altitude:', -0.061)
Latitude: 28.382108, Longitude: 36.483022
Roll: -0.02, Pitch: -0.05, Yaw: 2.68
('Altitude:', 0.502)
Latitude: 28.382108, Longitude: 36.483022
Roll: -0.03, Pitch: -0.00, Yaw: 2.77
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 2.171)
Latitude: 28.382107, Longitude: 36.483021
Roll: -0.03, Pitch: 0.04, Yaw: -3.14
('Altitude:', 2.956)
Latitude: 28.382109, Longitude: 36.483021
Roll: -0.01, Pitch: -0.02, Yaw: 2.90
('Altitude:', 3.013)
Latitude: 28.382110, Longitude: 36.483020
Roll: -0.06, Pitch: 0.00, Yaw: -2.97
Reached target altitude
Takeoff complete
Flying to waypoint: (28.382081, 36.482996)
Traceback (most recent call last):
  File "real_test_p2p_1.py", line 76, in <module>
    distance = vehicle.location.global_relative_frame.distance_to(target_location)
AttributeError: 'LocationGlobalRelative' object has no attribute 'distance_to'
pi@raspberrypi:~/droneTest $ python real_test_p2p_2.py
Connecting to vehicle on: /dev/ttyAMA0
Basic pre-arm checks
Starting ...
Arming motors
Waiting for arming...
WARNING:autopilot:Terrain: clamping offset 19 to 15
Taking off!
('Altitude:', -0.012)
Latitude: 28.382090, Longitude: 36.483018
Roll: -0.00, Pitch: 0.01, Yaw: 2.14
('Altitude:', -0.047)
Latitude: 28.382090, Longitude: 36.483018
Roll: -0.00, Pitch: 0.00, Yaw: 2.14
('Altitude:', -0.071)
Latitude: 28.382089, Longitude: 36.483018
Roll: 0.00, Pitch: 0.00, Yaw: 2.14
('Altitude:', 0.206)
Latitude: 28.382088, Longitude: 36.483019
Roll: -0.06, Pitch: 0.04, Yaw: 2.29
WARNING:autopilot:EKF3 IMU1 MAG0 ground mag anomaly, yaw re-aligned
WARNING:autopilot:EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned
('Altitude:', 0.898)
Latitude: 28.382087, Longitude: 36.483019
Roll: -0.03, Pitch: 0.06, Yaw: 2.77
('Altitude:', 0.976)
Latitude: 28.382088, Longitude: 36.483018
Roll: -0.01, Pitch: 0.01, Yaw: 2.82
('Altitude:', 0.996)
Latitude: 28.382089, Longitude: 36.483018
Roll: 0.01, Pitch: -0.02, Yaw: 2.73
Reached target altitude
Takeoff complete
Moving to the right
Traceback (most recent call last):
  File "real_test_p2p_2.py", line 76, in <module>
    distance = vehicle.location.global_relative_frame.distance_to(right_location)
AttributeError: 'LocationGlobalRelative' object has no attribute 'distance_to'

```

## Object Tracking
Since the drone is successfully running code without any errors, we can now move on to the next step which is object tracking. I have found this really nice (guide to autonomous object tracking)[https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/basic-object-tracking]