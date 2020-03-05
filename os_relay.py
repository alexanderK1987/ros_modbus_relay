import re
from pymodbus.client.sync import ModbusTcpClient as MbClient
import threading
import time
import rostopic
import subprocess

NODE_NAME = 'ROS_MODBUS_RELAY'
PUB_TOPIC = 'modbus_tcp_data'
SUB_TOPIC = 'modbus_tcp_cmd'
CMD_PATTERN = '(READ|WRITE)\s+(\w+)\s+(COUNT)\s+(\w+)(\s*[\w|\W]*){0,1}'
MODBUS_HOST = '192.168.10.30'
MODBUS_PORT = 502
MODBUS_TIMEOUT = 7e-3

class msg_handler():
    def __init__(self, client):
        self.client = client
        self.proc = subprocess.Popen(['rostopic', 'echo', '/'+SUB_TOPIC], stdout=subprocess.PIPE)
        self.prog = re.compile(CMD_PATTERN)
        self.t_thread = threading.Thread(target=self.start)
        self.kill = False
        self.t_thread.setDaemon(True)
        self.t_thread.start()
        self.__agv_stop__ = True

    def start(self):
        while self.kill == False:
            data = self.proc.stdout.readline()
            if not data:
                break

            if data.startswith("data:"):
                self.process(data.split('"')[1])
            time.sleep(1e-3)

    def stop(self):
        if not self.kill:
            print ('unsubscribing')
            self.kill = True
            self.proc.kill()

    def process(self, line):
        #print ('recv: %s' % line)
        m = self.prog.match(line)
        if m == None:
            print ('invalid command %s' % line)
            return
        cmd, addr, count = m.group(1), eval(m.group(2)), int(m.group(4))
        if cmd == 'WRITE':
            values = map(eval, m.group(5).split())
            values = [v&0x0000ffff for v in values]
            if (values[1] == 0) and (values[2] == 0): 
                if (not self.__agv_stop__):
                    self.__agv_stop__ = True
                    values[0] = 2
                else:
                    return
            else:
                self.__agv_stop__ = False
            if len(values) != count:
                print ('incorret amount of data')
                return
            if not self.client.is_socket_open():
                self.client.connect()
            self.async_write(addr, values)
            return
            
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
        print ('start checking HB')

    def __start__(self):
        while not self.kill:
            try:
                time.sleep(self.TIMEOUT)
                topic = rostopic.get_topic_class('/modbus_tcp_cmd')
                self.ros_alive = False if topic[0] == None else True
            except:
                self.ros_alive = False
    
if __name__  == '__main__':
    client = MbClient(MODBUS_HOST, port=MODBUS_PORT, timeout=MODBUS_TIMEOUT)
    client.connect()
    hbc = HbChecker()
    mh = None
    while True:
        try:
            while not hbc.ros_alive:
                print ('reconnecting')
                if mh != None and mh.kill == False:
                    mh.stop()
                time.sleep(hbc.TIMEOUT)
        except KeyboardInterrupt:
            break

        #print ('ok!')
        mh = msg_handler(client)

        try:
            while hbc.ros_alive:
                time.sleep(hbc.TIMEOUT)
        except KeyboardInterrupt:
            break

    print ('Terminating')
    hbc.kill = True
    if mh != None:
        mh.stop()
    client.close()

