#!/usr/bin/env python
import socket
from time import ctime
 
 
host='0.0.0.0'
port=812 # receive data
bufsize=1024
addr=(host, port)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(addr)	# bind socket at port
sock.listen(5)
while True:
    print('waiting for connection, ready to receive data from client...')
    clisock, cliaddr = sock.accept()	# waiting for new data
    print('connected from', cliaddr)
    print('peername:', clisock.getpeername())
    data = clisock.recv(bufsize).decode(encoding='utf_8', errors='strict') # receive and decode
    if not data:
        continue
    fin=open('message.txt',a) # append at the the file
    fin.write(data)
	fin.close()
    clisock.close() # close this connection and waiting for next one
    print('write new data at time: %s'% (ctime()) )
sock.close()
