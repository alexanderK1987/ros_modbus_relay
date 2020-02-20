import rospy
import re
from std_msgs.msg import String
from pymodbus.client.sync import ModbusTcpClient as MbClient
import threading
import time

NODE_NAME = 'ROS_MODBUS_RELAY'
PUB_TOPIC = 'modbus_tcp_data'
SUB_TOPIC = 'modbus_tcp_cmd'
CMD_PATTERN = '(READ|WRITE)\s+(\w+)\s+(COUNT)\s+(\w+)(\s*\[[\w|\W]*\]){0,1}'
MODBUS_HOST = '192.168.10.30'
MODBUS_PORT = 502
MODBUS_TIMEOUT = 7e-3

class msg_handler():
    def __init__(self, client, ros_publisher):
        self.client = client
        self.ros_publisher = ros_publisher
        self.prog = re.compile(pat)

    def callback(self, data):
        line = str(data.data)
        m = self.prog.match(line)
        if m == None:
            print ('invalid command %s' % line)
            return
        cmd, addr, count = m.group(1), eval(m.group(2)), int(m.group(4))
        if cmd == 'READ':
            values = self.read(addr, count)
            if values == None or len(values) == 0:
                print ('read error')
                return
            self.ros_publisher.publish(str(values))    
        elif cmd == 'WRITE':
            values = eval(m.group(5))
            if len(values) != count:
                print ('incorret amount of data')
                return
            self.async_write(addr, values)
            return
            
    def read(self, addr, count, unit=1):
        if not self.client.is_socket_open():
            return None
        rq = self.client.read_holding_registers(addr, count, unit=unit)
        return rq.registers

    def async_write(self, addr, count, unit=1):
        if not self.client.is_socket_open():
            return 
        t = threading.Thread(target=self.__async_write__, args=(addr,count,), kwargs={'unit':unit})
        t.setDaemon(True)
        t.start()

    def __async_write__(self, addr, values, unit=1):
        self.client.write_registers(addr, values, unit=unit)

if __name__  == '__main__':
    try:
        client = MbClient(MODBUS_HOST, port=MODBUS_PORT, timeout=MODBUS_TIMEOUT)
        client.connect()
        rospy.init_node(NODE_NAME)
        pub = rospy.Publisher(PUB_TOPIC, String, queue_size=64)
        handler = msg_handler(client, pub)
        rospy.Subscriber(SUB_TOPIC, String, handler.callback)
        rospy.spin()

    except KeyboardInterrupt:
        client.close()
