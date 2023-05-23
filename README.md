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

