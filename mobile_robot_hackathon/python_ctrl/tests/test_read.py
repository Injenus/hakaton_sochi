import serial
import time
import struct

com_port = 'COM7'
baud_rate = 1000000


rec_data = [-6 for i in range(24)] #
rec_dict = {0: "left_wh",1: "right_wh",2: "mode_move",3:'x_arm',4:'y_arm', 5:'z_arm',
            6:'mode_arm', 7: 'ax', 8:'ay', 9:'az', 10:'gx', 11:'gy', 12:'gz',
            13:'ang_x', 14:'ang_y', 15:'ang_z', 16:'odo_l', 17:'odo_r',
            18:'lidar_angle', 19:'lidar_dist', 20:'sonar_1', 21:'sonar_2',
            22:'ir', 23:'end_sens'}

send_flag = False
receive_flag = False
is_new_pack = False
receive_data_byte = bytearray()
rec_ind = 0

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

def get_int8(byte):
    # int8 = -2
    # int8 = struct.unpack('b', byte)[0]
    # return int8
    return byte

def get_int16(bytes):
    int16 = -2
    int16 = int.from_bytes(bytes,  "little", signed="True")
    return int16

def label_data(data):
    global rec_dict, rec_data
    # for i, el in enumerate(data):
    #     print(rec_dict.get(i, 'ZHOPA'), ':', el)
    for i, el in enumerate(rec_data):
        print(rec_dict.get(i, 'ZHOPA'), ':', el)

def parsing(byte_arr):
    global rec_data
    loc_rec_data = [-3 for i in range(24)]
    #print("PARS", len(byte_arr), byte_arr)
    #print("init pars")

    for i in range(22):
        loc_rec_data[i] = get_int16(byte_arr[i * 2:i * 2 + 2])
        #print(i, rec_data[i])

    loc_rec_data[22] = get_int8(byte_arr[44])
    #print(rec_data[22])
    loc_rec_data[23] = get_int8(byte_arr[45])


    if loc_rec_data[7] == -123 and loc_rec_data[12] == -12345:
        return loc_rec_data
    else:
        return rec_data

def real_rec_data():
    global send_flag, is_new_pack, rec_ind, receive_data_byte, rec_data
    buff = ser.read_all()
    # print('readed', buff, len(buff))
    # print()
    #print(buff, len(buff))
    if not is_new_pack:
        rec_ind = 0
        for byte in buff:

            # try:
            #     if byte.decode() == '%':
            #         is_new_pack = True
            #         print('TRUEEE')
            #     else:
            #         print(byte, 'qqqqq',byte.decode())
            # except:
            #     print("ERR byte.decode")
            #     pass

            if byte == 37:
                is_new_pack = True
                #print('TRUEEE')
            else:
                #print(byte)
                pass

            rec_ind += 1
        if is_new_pack:
            # receive_data_byte = buff[rec_ind-32:]
            receive_data_byte = buff[rec_ind:]
    else:
        receive_data_byte += buff
    #print('Cur db: ',receive_data_byte, len(receive_data_byte), rec_ind)

    if len(receive_data_byte) >= 48:
        #print(receive_data_byte)

        
        receive_data_byte = receive_data_byte[2:48]
        #print('split_rec_d', receive_data_byte, len(receive_data_byte))
       # try:
        rec_data = parsing(receive_data_byte)
        print('Received:', rec_data)
        label_data(rec_data)
        #except:
            #print('Fail!! (last Received):', rec_data)

        is_new_pack = False
        send_flag = True

while (1):
    real_rec_data()
    time.sleep(0.04)
    