import socket
import time

#UDP_IP = "192.168.0.144"
UDP_IP = "192.168.1.144"
SHARED_UDP_PORT = 4210

# create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
sock.connect((UDP_IP, SHARED_UDP_PORT))

c32 = lambda x: int(x[0])+256*int(x[1]) + 256*256*int(x[2])+256*256*256*int(x[3])
c16 = lambda x: int(x[0])+256*int(x[1])

def loop():
    n = 0
    while True:
        n += 1
        if n%2 == 0:
            ping()
        data = sock.recv(2048)
        # the structure of the received packet
        # ts   4b
        # id   4b
        # voltage   4b
        # sampla list  4+2+2b
        ts = c32(data[0:4])
        cntr = c32(data[4:8])
        volt = c32(data[8:12])
        for p in range(12, 524, 8):
            print (ts, cntr, volt, c32(data[p:p+4]), c16(data[p+4:p+6]), c16(data[p+6:p+8]), flush=True)

def ping():
    sock.send('ping'.encode())  # make sure uC knows I am still alive

if __name__ == "__main__":
    sock.send('start'.encode())  # ask for sensory data
    loop()



