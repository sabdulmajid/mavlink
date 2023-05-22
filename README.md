# MAVLink 
## Code explanation
The ```mavgen.py``` is a Python script that generates a Python implementation of a MAVLink protocol XML file. MAVLink is a protocol for communication between unmanned aerial vehicles (UAVs) and ground control stations. The XML files that define the MAVLink protocol specify the names, types, and sizes of the messages that can be sent between UAVs and ground control stations.

The Python script works by first parsing the MAVLink XML files. This is done using the mavparse module, which is a Python library for parsing MAVLink XML files. Once the XML files have been parsed, the script generates a Python implementation of the MAVLink protocol. The generated Python implementation includes classes for each of the messages defined in the MAVLink XML files. The classes provide methods for sending and receiving messages, as well as methods for getting and setting the properties of the messages.

The Python script also validates the MAVLink XML files. This is done by comparing the XML files to the MAVLink schema, which is a XML document that defines the syntax of MAVLink XML files. If the XML files do not conform to the schema, the script will print an error message.

The Python script is a useful tool for developers who want to create applications that communicate with UAVs using the MAVLink protocol. The script can be used to generate Python implementations of MAVLink XML files, which can then be used to send and receive messages between UAVs and ground control stations.

Here is a more detailed explanation of the steps that the Python script takes to generate a Python implementation of a MAVLink protocol XML file:

  1. The script imports the mavparse module.
  2. The script parses the MAVLink XML file using the ```mavparse.MAVXML``` class.
  3. The script creates a Python class for each message defined in the MAVLink XML file.
  4. The script adds methods to the Python classes for sending and receiving messages.
  5. The script adds methods to the Python classes for getting and setting the properties of the messages.
  6. The script validates the MAVLink XML file against the MAVLink schema.
  7. The script prints an error message if the ```MAVLink XML``` file does not conform to the schema.
  8. The script returns the Python classes.
    
After that, a function called ```mavgen_python_dialect()``` is defined. This function generates Python code for a MAVLink dialect. The function takes three arguments:

```dialect```: The name of the MAVLink dialect.

```wire_protocol```: The MAVLink wire protocol version.

```with_type_annotations```: A boolean value indicating whether to generate type annotations for the Python code.

The function first gets the path to the MAVLink dialect directory. It then gets the path to the MAVLink dialect XML file. If the XML file does not exist, the function tries to get the path to the XML file from the message definitions directory.

The function then creates an Opts object. This object contains the following information:

The output directory.
The MAVLink wire protocol version.
The language to generate the Python code in.
Whether to generate type annotations for the Python code.
The function then checks if the Python 2 or Python 3 version of the mavgen script is available. It then calls the mavgen script to generate the Python code.

The function finally returns a boolean value indicating whether the Python code was generated successfully.

The mavgen script is a Python script that generates Python code for a MAVLink dialect. The script takes two arguments:

```output```: The output directory.
```xml```: The path to the MAVLink dialect XML file.

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
The 'mavutil' module provides a function, ```mavutil_connection()``` which always a user to connect with a device. Example code to connect to a UDP port:

```python
from pymavlink import mavutil
connection = mavutil.mavlink_connection('udpin:localhost:14540')
connection.wait_heartbeat()
print("Heartbeat received from the system (system %u component %u)" % (connection.target_system, connection.target_component))
```

```udpin``` creates a socket to listen for a UDP connection, whereas, ```udpout``` creates a socket that iniatiates a UDP connection.
```wait_heartbeat()``` waits for a heartbeat message from the system. This is a blocking function, so it will wait until a heartbeat is received.
Other useful ```mavlink_connection()``` parameters: ```source_system (default 255)```, ```source_component (default 0)``` and ```dialect (default ArduPilot)```

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
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
      # Or use the following method if it is from a MAVLink application
      connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    ```
    