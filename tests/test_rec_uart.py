import serial

com_port = 'COM3'
baud_rate = 115200

is_rec = True
ser = serial.Serial(com_port, baud_rate)

def split_bytes(input_bytes):
    return [byte for byte in input_bytes]

while is_rec:

    received_data = ser.readline()

    if received_data[-1] == 10 and received_data[-2] == 13:
        data = received_data[:-2]
        print(data)
        # value = struct.unpack('H', received_data)[0]
        # print(f'Received value: {value}')


