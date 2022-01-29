import smbus
import time

bus = smbus.SMBus(1)
SLAVE_ADDRESS = 0x04

def request_data():
        read_value = int(bus.read_byte(SLAVE_ADDRESS))
        print(read_value)

while True:
        #WRITE DATA
        bus.write_byte(SLAVE_ADDRESS, ord('1'))
        time.sleep(1)
        request_data()























