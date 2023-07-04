import socket
import threading
import keyboard
from robot import Robot


# Get data from user
def handle_client(client_socket):
    while True:
        data = client_socket.recv(1024)
        if keyboard.is_pressed('w'):
            client_socket.close()
            return
        if not data:
            break
        print(data.decode())
        response = "Response from server"
        received = data.decode().split(' ')
        robot = Robot()
        print(received)
        robot.setSpeed(int(received[0]))
        robot.holdAng(int(received[1]))
        client_socket.send(response.encode())

    client_socket.close()


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.88.92', 8888))
server_socket.listen(10)

pending_connections = []


# Accept connection and adding into the array
def accept_connections():
    while True:
        client_socket, address = server_socket.accept()
        pending_connections.append(client_socket)
        print("Pending connections:")
        for i, conn in enumerate(pending_connections):
            print(f"{i + 1}. {conn.getpeername()}")


# Managing connections
def process_connections():
    while True:
        choice = input("Enter the index of the connection to process (or 'q' to quit): ")
        if choice.lower() == 'q':
            break

        try:
            index = int(choice) - 1
            if index < 0 or index >= len(pending_connections):
                print("Invalid choice. Try again.")
            else:
                client_socket = pending_connections.pop(index)
                client_thread = threading.Thread(target=handle_client, args=(client_socket,))
                client_thread.start()
        except ValueError:
            print("Invalid choice. Try again.")
    server_socket.close()


accept_thread = threading.Thread(target=accept_connections)
accept_thread.start()

process_thread = threading.Thread(target=process_connections)
process_thread.start()

accept_thread.join()
process_thread.join()
