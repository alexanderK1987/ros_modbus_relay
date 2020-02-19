# ROS-Modbus TCP Relay Service
A dedicated service that take modbus command from ROS to modbusTCP.

# Format Example
## Read registers
`READ 0x1620 COUNT 4`
## Write registers
`WRITE 0x1620 COUNT 3 [1, 100, 200]`
