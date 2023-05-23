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
  - **OPEN_DRONE_ID_SYSTEM_UPDATE**:	A subset of the **OPEN_DRONE_ID_SYSTEM** message, containing only the fields that must be updated at a high rate. Typically sent from the GCS to provide data to the RID transmitter component. If both **OPEN_DRONE_ID_SYSTEM** and **OPEN_DRONE_ID_SYSTEM_UPDATE** are used, the more efficient **OPEN_DRONE_ID_SYSTEM_UPDATE** will be used at a high rate and the full **OPEN_DRONE_ID_SYSTEM** at a low rate, to reduce the traffic on the control link.

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

For future reference when simulating: 
  - [DroneKit-SITL documentation](https://github.com/dronekit/dronekit-sitl)
  - [Running examples with DroneKit](https://dronekit-python.readthedocs.io/en/latest/examples/running_examples.html)
  - [GitHub Repository with useful examples](https://github.com/dronekit/dronekit-python/tree/master/examples)
  