import socket

HOST = '192.168.1.15'  # Dirección IP del localhost
PORT = 80  # Puerto para escuchar

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    print(f"Servidor escuchando en {HOST}:{PORT}")
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Conectado por', addr)
        message = b'EL SENOR ES MI PASTOR Y NADA ME FALTARA'
        for i in range(10):
            conn.sendall(message)
            print(f"Mensaje {i+1} enviado al cliente: {message.decode()}")