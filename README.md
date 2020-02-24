# ROS-Modbus TCP Relay Service
A dedicated service that take modbus command from ROS to modbusTCP.
This service should be run after the main ROS application is launched.

This service subscribes to ROS topic `/modbus_tcp_cmd` and detects incomming commands.
If the incomming message is a read command, the service publishs the modbus response to ROS topic `/modbus_tcp_data`.

## Requirements
This service requires `rospy`, `pymodbus` to run.
The installation command should be:

```bash
sudo pip install -r requirements.txt
```

## Protocol Data Format
### Read registers
The read commands must follow the following format:

 - `READ [register address] COUNT [# of registers]`

For example:

 - `READ 0x0426 COUNT 4`

If the read command is issued successfully, the response is then publish to `/modbus_tcp_data`:

 - `[1, 2341, 1, 3122]`

### Write registers
The write commands must follow the following format:

 - `WRITE [register address] COUNT [# of regiser] [list of unsigned-16-bit integer (uint_16[])]`

For example:

 - `WRITE 0x1620 COUNT 3 1 100 200`

