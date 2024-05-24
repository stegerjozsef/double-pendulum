import socket
import time

UDP_IP = "192.168.0.144"
SHARED_UDP_PORT = 4210

# create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
sock.connect((UDP_IP, SHARED_UDP_PORT))

def loop():
    n = 0
    with open('/tmp/pdump.bin', 'wb') as f:
        while True:
            n += 1
            if n%100 == 0:
                ping()
            data = sock.recv(2048)
            f.write(data)
            print (
                    len(data), 
                    int(data[0])+256*int(data[1]),
                    int(data[2])+int(data[3])*256+
                    int(data[4])*256*256+int(data[5])*256*256*256,
                    data[5],
                    data[6]
                    )

def ping():
    sock.send('ping'.encode())  # make sure uC knows I am still alive

if __name__ == "__main__":
    sock.send('start'.encode())  # ask for sensory data
    loop()



