import socket
import select

HOSTS = ["10.0.0.169", "10.0.0.170", "10.0.0.171"]
PORTS = [42069, 42070, 42071]
if __name__ == "__main__":
    print("Creating remote server...")
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM), socket.socket(socket.AF_INET, socket.SOCK_DGRAM), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]
    for i in range(len(sockets)):
        sockets[i].bind((HOSTS[i], PORTS[i]))
        sockets[i].listen()
        while True:
            ready_sockets,_,_ = select.select(sockets, [], [])
            for e, s in enumerate(ready_sockets):
                data, addy = sockets[e].recvfrom(1024)

    print("Finished socket setup...")
