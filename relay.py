import rospy
import re
from std_msgs.msg import String
from pymodbus.client.sync import ModbusTcpClient as MbClient
import threading
import time
import rostopic

NODE_NAME = 'ROS_MODBUS_RELAY'
PUB_TOPIC = 'modbus_tcp_data'
SUB_TOPIC = 'modbus_tcp_cmd'
CMD_PATTERN = '(READ|WRITE)\s+(\w+)\s+(COUNT)\s+(\w+)(\s*[\w|\W]*){0,1}'
MODBUS_HOST = '192.168.10.30'
MODBUS_PORT = 502
MODBUS_TIMEOUT = 7e-3

class msg_handler():
    def __init__(self, client, ros_publisher):
        self.client = client
        self.ros_publisher = ros_publisher
        self.prog = re.compile(CMD_PATTERN)

    def callback(self, data):
        line = str(data.data)
        m = self.prog.match(line)
        if m == None:
            print ('invalid command %s' % line)
            return
        cmd, addr, count = m.group(1), eval(m.group(2)), int(m.group(4))
        if cmd == 'READ':
            if not self.client.is_socket_open():
                self.client.connect()
            values = self.client.read(addr, count)
            if values == None or len(values) == 0:
                print ('read error')
                return
            self.ros_publisher.publish(str(values))

        elif cmd == 'WRITE':
            values = map(eval, m.group(5).split())
            values = [v&0x0000ffff for v in values]
            if (values[1] == 0) and (values[2] == 0):
                values[0] = 2
            if len(values) != count:
                print ('incorret amount of data')
                return
            if not self.client.is_socket_open():
                self.client.connect()
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
        try:
            self.client.write_registers(addr, values, unit=unit)
        except:
            pass

class HbChecker():
    def __init__(self):
        self.TIMEOUT = 1.0 # seconds
        self.kill = False
        self.ros_alive = False
        self.t_checker = threading.Thread(target=self.__start__)
        self.t_checker.setDaemon(True)
        self.t_checker.start()

    def __start__(self):
        while not self.kill:
            try:
                time.sleep(self.TIMEOUT)
                topic = rostopic.get_topic_class('/modbus_tcp_cmd')
                if topic[0] == None:
                    self.ros_alive = False
                else:
                    self.ros_alive = True
            except:
                self.ros_alive = False
    
if __name__  == '__main__':
    handler = None
    client = MbClient(MODBUS_HOST, port=MODBUS_PORT, timeout=MODBUS_TIMEOUT)
    client.connect()
    hbc = HbChecker()
    KILL = False
    while not KILL:
        try:
            while hbc.ros_alive == False and (KILL == False):
                print ('dead')
                time.sleep(hbc.TIMEOUT)
        except KeyboardInterrupt:
            print ('SIGTERM.')
            hbc.kill = True
            KILL = True

        if not KILL:
            print ('re/connecting')
            rospy.init_node(NODE_NAME)
            pub = rospy.Publisher(PUB_TOPIC, String, queue_size=64)
            handler = msg_handler(client, pub)
            rospy.Subscriber(SUB_TOPIC, String, handler.callback)

        try:
            while hbc.ros_alive and (KILL == False):
                print ('online')
                time.sleep(hbc.TIMEOUT)
        except KeyboardInterrupt:
            print ('SIGTERM.')
            hbc.kill = True
            KILL = True
            break

    print ('Terminating')
    handler.kill = True
    client.close()

