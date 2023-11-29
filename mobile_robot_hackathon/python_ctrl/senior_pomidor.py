import serial
import time
import struct

## ЭТО БЕЗ ПОТОКОВ!!!!!!!!!!!!!!!!!!!!

com_port = 'COM7'
baud_rate = 1000000
try:
    ser = serial.Serial(com_port, baud_rate, timeout=1)
except:
    print("ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    exit()

while (not ser.isOpen()):
    print("Try send Ser")
    ser.open()
    time.sleep(1) 
print("Ser is Open!")      
    
##### send blok ####
flag_start = True
flag_finish = False
data_send = [0, 0, 110, 110, 110, 90, 0]
init_sb = '#'
send_flag = False
receive_flag = False
#########################

### rec block ########
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
##############
is_moving_now = 'hz'
flag_done = [-1, True]

def send_data(data_to_send):
    global send_flag, flag_done

    if (send_flag):
        byte_data = bytearray(struct.pack('bbhhhbb', *data_to_send[:2], *data_to_send[2:6], *data_to_send[6:]))  
        send_pack = init_sb.encode('utf-8') + struct.pack('B',hash(byte_data)) + byte_data
        #print('\nHash-sum is', send_pack[1])
        #print('Sended',send_pack)
        ser.write(send_pack)
        send_flag = False


def close_serial():
    pass

def go_stop():
    global data_send, flag_done
    data_send[0] = 0
    data_send[1] = 0
    send_data(data_send)
    flag_done[0] = 10 + 0
    flag_done[1] = False
    


def go_forw(move_time): # time = time*100 ms!!!!
    global data_send, flag_done
    if move_time > 127:
        move_time = 127
    if move_time < 0:
        move_time = 0
    data_send[0] = 1
    data_send[1] = move_time
    send_data(data_send)
    data_send[0] = 0
    data_send[1] = 0
    print('Forw, sec: ', move_time/10)
    flag_done[0] = 1 + 10
    flag_done[1] = False

def go_back(move_time):
    global data_send, flag_done
    if move_time > 127:
        move_time = 127
    if move_time < 0:
        move_time = 0
    data_send[0] = 2
    data_send[1] = move_time
    send_data(data_send)
    data_send[0] = 0
    data_send[1] = 0
    print('Backw, sec: ', move_time/10)
    flag_done[0] = 2 + 10
    flag_done[1] = False

def go_turn(type, angle):
    global data_send, flag_done
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
    if type == 4:
        print('right wh')
    if type == 5:
        print('left wh')
    flag_done[0] = 3 + 10
    flag_done[1] = False

def q1_man(q1):
    global data_send, flag_done
    data_send[2] = q1
    print("q1 is", q1)


def hash(byte_data):
    crc = 0 
    for byte in byte_data:
        crc = (crc << 3) & (0xff) | byte
        crc = (crc << 4) & (0xff) | byte
    return crc

#####################################################
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
    global send_flag, is_new_pack, rec_ind, receive_data_byte, rec_data, is_moving_now
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
        # print('Received:', rec_data)
        # label_data(rec_data)
        #except:
            #print('Fail!! (last Received):', rec_data)

        is_new_pack = False
        send_flag = True
        if rec_data[2] == 0:
            is_moving_now = 'yes'
        elif rec_data[2] == 1:
            is_moving_now = 'no'
        else:
            is_moving_now = 'hz'
####################################################################3
def change_func(ctrl_pack):
    if ctrl_pack[0] == 1:
        go_forw(ctrl_pack[1])

    elif ctrl_pack[0] == 2:
        go_back(ctrl_pack[1])

    elif 2 < ctrl_pack[0] < 6:
        go_turn(ctrl_pack[0], ctrl_pack[1])


def sens_aruco():
    id = 0
    return id

def sens_ovosch():
    type = 0 # 0 - pomid, 1- baclagzh
    return type

one_cell = 70  #70
cell_double = [35, 35]
cell_triple = [23, 23, 23]
ang = 90 #90
stab = 0

    
path_etal= [[3, -90],
    [1, 23], # near odocsh
            [3, 0],
            [3, 0],
        [1, 69],
        [3, 0],
            [3, 0],
        [1, 69], 
        [3, -90],
        [1, 58],
        [3, 0],
            [3, 0],
        [1, 58],
        [3, -90], #near ovosch
        [1, 70],
        [3, 0],
            [3, 0],
        [1, 70],
        [3, 0],
            [3, 0],
        [1, 23],
        [3, 90],
        ] 
path_test = [[1,one_cell//10], [3,90]]
path = [
    [1, one_cell//2],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [3, 90],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [3, 90],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//3],
    [3, 0],
    [1, one_cell//2],
    [3, -90],
]


t_rec =[0, 0.2]
t_send = [0, 1]

start_id = -1
finish_id = -1
#####################################
count = 0
time.sleep(5)
while (not flag_finish):
    while (start_id == -1):
        start_id = sens_aruco()

    if (time.time() - t_rec[0] > t_rec[1]):
        t_rec[0] = time.time()
        real_rec_data()
        # print('\n\n################# ПРИНЯЛИ ################################\n\n')
        # print(is_moving_now)
    #time.sleep(0.1)
    
    if (time.time() - t_send[0] > t_send[1]):
        t_send[0] = time.time()
        if is_moving_now == 'no' and flag_done[1]:
            if count < len(path_etal):
                type_ovosch = sens_ovosch()
                if  type_ovosch == 0: #pomidor
                    q1_man(90)
                elif type_ovosch == 1:
                    q1_man(175)
                else:
                    q1_man(175)
                change_func(path_etal[count])
                print("№№№№№№№№№№№ СДЕДЛАЛИ №№№№№№№№№№№", count)
                # is_moving_now = 'hz'
                count += 1 
            else:
                while (finish_id == -1):
                    finish_id = sens_aruco()
                print('finish')
                print('start aruco is {}, fnish aruco is {}'.format(start_id, finish_id))

    if rec_data[20] == flag_done[0]:
        flag_done[1] = True
        
        






