# ROS-Modbus TCP Relay Service
A dedicated service that take modbus command from ROS to modbusTCP.
Should be run after the main ROS application is launched.
This service subscribes to ROS topic `/modbus_tcp_cmd` and detect for incomming commands.
If the message is a read command, the service publish the modbus response to ROS topic `/modbus_tcp_data`.

# Protocol Data Format
## Read registers
The read command must follow the format:

`READ [register address] COUNT [# of registers]`

For example:

`READ 0x0426 COUNT 4`

If the read command is issued successfully, the response is then publish to `/modbus_tcp_data`:

`[1, 2341, 1, 3122]`

## Write registers
`WRITE 0x1620 COUNT 3 [1, 100, 200]`

