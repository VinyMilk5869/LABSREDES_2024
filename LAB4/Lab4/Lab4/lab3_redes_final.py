import socket

# Dirección IP y puerto del servidor Modbus
SERVER_IP = '192.168.164.211'  # Cambia por la dirección IP del ESP32
SERVER_PORT = 502

def entero_a_bytes1(numero):
    # Convertir entero a bytes con longitud de 1 bytes (8 bits)
    bytes_resultantes = numero.to_bytes(1, byteorder='big')
    return bytes_resultantes

def entero_a_bytes2(numero):
    # Convertir entero a bytes con longitud de 2 bytes (16 bits)
    bytes_resultantes = numero.to_bytes(2, byteorder='big')
    return bytes_resultantes

def build_modbus_request(transaction_id,unit_id, function_code,position1,position2=None,data=None):
    
    protocol_id = 0  # Modbus TCP generalmente usa 0
    
    # Construir la trama de solicitud Modbus

    # Si la funcion es de escritura

    if data is not None:
        transaction_id_bytes = entero_a_bytes2(transaction_id)
        protocol_id_bytes = entero_a_bytes2(protocol_id)
        unit_id_bytes = entero_a_bytes1(unit_id)
        function_code_bytes = entero_a_bytes1(function_code)
        position1_bytes = entero_a_bytes2(position1)
        data_bytes = entero_a_bytes2(data)
        length = len(function_code_bytes) + len(position1_bytes) + len(data_bytes)
        length_bytes = entero_a_bytes2(length)

        # Concatenar todos los bytes

        request = transaction_id_bytes + protocol_id_bytes + length_bytes + unit_id_bytes + function_code_bytes + position1_bytes + data_bytes

    else:
        # Si la función es de lectura
        transaction_id_bytes = entero_a_bytes2(transaction_id)
        protocol_id_bytes = entero_a_bytes2(protocol_id)
        unit_id_bytes = entero_a_bytes1(unit_id)
        function_code_bytes = entero_a_bytes1(function_code)
        position1_bytes = entero_a_bytes2(position1)
        position2_bytes = entero_a_bytes2(position2)
        length = len(function_code_bytes) + len(position1_bytes) + len(position2_bytes) + len(unit_id_bytes)
        length_bytes = entero_a_bytes2(length)

        # Concatenar todos los bytes
        request = transaction_id_bytes + protocol_id_bytes + length_bytes + unit_id_bytes + function_code_bytes + position1_bytes + position2_bytes
    
    return request
def mostrar_menu():
    print("1. Leer registros")
    print("2. Modificar Registro")

# Definir las funciones correspondientes a cada opción
def leer_coil(transaction_id):
    position = int(input("Coil que desea leer: "))
    trama = build_modbus_request(transaction_id,1,1,position)
    return trama  # Código para la lectura de coil


def leer_registros(transaction_id):
    position1 = int(input("Primer registro a leer: "))
    position2 = int(input("Ultimo registro a leer: "))
    trama = build_modbus_request(transaction_id,1,3,position1,position2)
    return trama  # Código para la lectura de registro


def modificar_coil(transaction_id):
    position = int(input("Coil que desea modificar: "))
    value = int(input("Nuevo estado del coil: "))
    trama = build_modbus_request(transaction_id,1,5,position,value)
    return trama  # Código para modificar coil

def modificar_registro(transaction_id):
    position = int(input("Qué registro desea modificar?: "))
    value = int(input("Nuevo valor para el registro: "))
    trama = build_modbus_request(transaction_id,1,6,position,value)
    return trama  # Código para modificar registro

def realizar_accion(opcion,transaction_id):
    switch = {
        1: leer_registros,
        2: modificar_registro,
    }
    
    # Obtener la función correspondiente a la opción seleccionada
    seleccion = switch.get(opcion, lambda: "Opción no válida")
    
    # Llamar a la función seleccionada
    resultado = seleccion(transaction_id)
    
    return resultado

def send_modbus_request(request_data):

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Conectar al servidor Modbus
        client_socket.connect((SERVER_IP, SERVER_PORT))

        # Enviar la solicitud al servidor
        client_socket.sendall(request_data)

        # Esperar y recibir la respuesta del servidor
        response = client_socket.recv(1024)

        return response

    finally:
        client_socket.close()


def interpretar_trama_modbus(trama):
    # Campos relevantes de la trama Modbus
    transaction_identifier = (trama[0] << 8) | trama[1]
    protocol_identifier = (trama[2] << 8) | trama[3]
    length = (trama[4] << 8) | trama[5]
    unit_identifier = trama[6]
    function_code = trama[7]
 
    # Verificar si es una función de lectura de múltiples registros (0x03)
    if function_code == 0x03:
        # Calcular el número de registros en la trama
        print(length)
        num_registers = (length - 2)//2 # Cada registro está representado por 2 bytes
 
        print(num_registers)
        # Iterar sobre los registros en la trama y mostrar sus valores
        for i in range(num_registers):
            register_value = (trama[8 + 2*i] << 8) | trama[9 + 2*i]
            register_bytes = register_value.to_bytes(2, byteorder='big')
            # Interpretar los bytes como un entero con signo
            signed_register_value = int.from_bytes(register_bytes, byteorder='big', signed=True)
            print(f"Registro {i}: {signed_register_value}")
            

    elif function_code == 0x06:  # escribir un solo registro1
        
        register_address = (trama[8] << 8) | trama[9]
        new_value = (trama[10] << 8) | trama[11]
        print("Registro", register_address, "modificado al valor", new_value)

    


 

def main(): 
  
    transaction_id = 0 
    try:
        while True:
            # Mostrar el menú al usuario
            mostrar_menu() 
            opcion = int(input("¿Qué opción desea realizar?"))
            trama = realizar_accion(opcion,transaction_id)
            print(f"Trama enviada: {trama}")
            response = send_modbus_request(trama)
            print(f"Trama recibida:{response}")
            interpretar_trama_modbus(response)
            transaction_id+=1
            

    except KeyboardInterrupt:
        print("Adiós!")

if __name__ == "__main__":
    main()  # Ejecutar la función principal
