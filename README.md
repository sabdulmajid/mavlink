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
