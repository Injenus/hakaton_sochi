import multiprocessing
import time

# Файл 1: neural_network.py
def neural_network_function(shared_variable):
    while True:
        # Ваш код для работы нейронной сети и записи переменных
        # Например: shared_variable.value = значение_из_нейронной_сети
        time.sleep(1)

# Файл 2: data_processing.py
def data_processing_function(shared_variable):
    while True:
        # Ваш код для обработки переменных и выполнения действий
        # Например: значение = shared_variable.value
        time.sleep(1)

# Файл 3: com_port_reader.py
def com_port_reader(shared_variable):
    while True:
        # Ваш код для чтения данных из COM-порта
        # Например: данные = читать_из_COM_порта()
        # shared_variable.value = данные
        time.sleep(1)

if __name__ == '__main__':
    shared_variable = multiprocessing.Value('i', 0)  # 'i' - тип данных, 0 - начальное значение
    process1 = multiprocessing.Process(target=neural_network_function, args=(shared_variable,))
    process2 = multiprocessing.Process(target=data_processing_function, args=(shared_variable,))
    process3 = multiprocessing.Process(target=com_port_reader, args=(shared_variable,))

    process1.start()
    process2.start()
    process3.start()
