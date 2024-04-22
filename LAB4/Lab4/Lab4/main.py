import socket

# Dirección IP del ESP32
IP_ESP32 = '192.168.246.3'
PORT = 502  # Puerto Modbus TCP/IP

def leer_registro(direccion_registro):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as cliente_modbus:
        cliente_modbus.connect((IP_ESP32, PORT))

        # Construir solicitud Modbus TCP/IP para leer un registro
        solicitud = b'\x00\x01\x00\x00\x00\x06\x01\x03' + \
                    direccion_registro.to_bytes(2, byteorder='big') + \
                    b'\x00\x01'  # Leer un solo registro

        cliente_modbus.sendall(solicitud)

        # Recibir respuesta del ESP32
        respuesta = cliente_modbus.recv(1024)

        # Analizar la respuesta para extraer el valor del registro
        valor = respuesta[9:11]  # La respuesta comienza desde el noveno byte hasta el undécimo byte
        valor = int.from_bytes(valor, byteorder='big')  # Convertir bytes a entero

        print(respuesta)
        
        

def escribir_registro(direccion_registro, nuevo_valor):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as cliente_modbus:
        cliente_modbus.connect((IP_ESP32, PORT))

        # Construir solicitud Modbus TCP/IP para escribir en un registro
        solicitud = b'\x00\x01\x00\x00\x00\x06\x01\x06' + \
                    direccion_registro.to_bytes(2, byteorder='big') + \
                    nuevo_valor.to_bytes(2, byteorder='big')

        cliente_modbus.sendall(solicitud)
        
        respuesta = cliente_modbus.recv(1024) 
        print(respuesta)
        

def main():
    cliente_modbus = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        cliente_modbus.connect((IP_ESP32, PORT))

        while True:
            opcion = input("¿Qué quieres hacer? (leer/escribir/salir): ").lower()

            if opcion == 'leer':
                
                direccion_registro = int(input("Ingrese la dirección del registro a leer: "))
                valor = leer_registro(direccion_registro)
                
                
            elif opcion == 'escribir':
                direccion_registro = int(input("Ingrese la dirección del registro a escribir: "))
                nuevo_valor = int(input("Ingrese el nuevo valor para el registro: "))
                escribir_registro(direccion_registro, nuevo_valor)
                print("Registro actualizado correctamente.")
                
            
            elif opcion == 'salir':
                break
            else:
                print("Opción no válida. Por favor, ingrese 'leer' o 'escribir'.")

    finally:
        cliente_modbus.close()

if __name__ == "__main__":
    main()
    