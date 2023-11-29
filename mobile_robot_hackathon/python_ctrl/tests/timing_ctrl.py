import serial
import time
import struct

com_port = 'COM7'
baud_rate = 1000000
try:
    ser = serial.Serial(com_port, baud_rate, timeout=0.08)
except:
    print("ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    exit()

while (not ser.isOpen()):
    print("Try send Ser")
    ser.open()
    time.sleep(1) 
print("Ser is Open!")      
    

flag_start = True
flag_finish = False

data_send = [0, 0, 110, 110, 110, 90, 0]
init_sb = '#'
send_flag = False
receive_flag = False

def send_data(data_to_send):
    global send_flag
    if (send_flag or True):
        byte_data = bytearray(struct.pack('bbhhhbb', *data_to_send[:2], *data_to_send[2:6], *data_to_send[6:]))  
        send_pack = init_sb.encode('utf-8') + struct.pack('B',hash(byte_data)) + byte_data
        #print('\nHash-sum is', send_pack[1])
        #print('Sended',send_pack)
        ser.write(send_pack)
        send_flag = False


def close_serial():
    pass

def go_stop():
    global data_send
    data_send[0] = 0
    data_send[1] = 0
    send_data(data_send)
    


def go_forw(move_time): # time = time*100 ms!!!!
    global data_send
    if move_time > 127:
        move_time = 127
    if move_time < 0:
        move_time = 0
    data_send[0] = 1
    data_send[1] = move_time
    send_data(data_send)
    data_send[0] = 0
    data_send[1] = 0
    print('Forw, sec: ', move_time//10)
    time.sleep(move_time//10 + 0.1)
    pass

def go_back(move_time):
    global data_send
    if move_time > 127:
        move_time = 127
    if move_time < 0:
        move_time = 0
    data_send[0] = 2
    data_send[1] = move_time
    send_data(data_send)
    data_send[0] = 0
    data_send[1] = 0
    print('Backw, sec: ', move_time//10)
    time.sleep(move_time//10 + 0.1)
    pass

def go_turn(type, angle):
    global data_send
    if angle > 127:
        angle = 127
    if angle < -127:
        angle = -127
    data_send[0] = type
    data_send[1] = angle
    send_data(data_send)
    data_send[0] = 0
    data_send[1] = 0
    print('Turn deg: ', angle)
    time.sleep(10)
    pass


def hash(byte_data):
    #crc = struct.pack('B', 0)
    crc = 0 
    for byte in byte_data:
        crc = (crc << 3) & (0xff) | byte
        crc = (crc << 4) & (0xff) | byte
    return crc

while (not flag_finish):

    go_forw(50)
    time.sleep(1)

    go_turn(3, 90)
    time.sleep(1)

    go_forw(20)
    time.sleep(1)

    go_turn(3, 90)
    time.sleep(1)


    go_forw(30)
    time.sleep(1)
    go_forw(20)
    time.sleep(1)

    go_turn(3, 90)
    time.sleep(1)

    go_forw(20)
    time.sleep(1)

    flag_finish = True



