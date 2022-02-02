import smbus
import time

# config variables
delay = 0.25 
bus = smbus.SMBus(1)
SLAVE_ADDRESS = 0x04

def request_data():
        read_value = int(bus.read_byte(SLAVE_ADDRESS))
        print(read_value)


while True:
    command = input("1: flag 1, 2: flag 2")

    if command == "1":
        bus.write_byte(SLAVE_ADDRESS, 1)
        time.sleep(1)
        for i in range(10):
            time.sleep(delay)
            request_data()

    elif command == "2":
        bus.write_byte(SLAVE_ADDRESS, 2)
        time.sleep(1)
        for i in range(40):
            time.sleep(delay)
            request_data()
