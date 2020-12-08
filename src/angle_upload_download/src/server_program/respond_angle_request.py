#!/usr/bin/env python
import socket
from time import ctime
 
 
host='0.0.0.0'
port=813 # receive data
bufsize=1024
addr=(host, port)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(addr)	# bind socket at port
sock.listen(5)
while True:
    print('waiting for connection, ready to respond to client request...')
    clisock, cliaddr = sock.accept()	# waiting for new data request from client
    print('connected from', cliaddr)
    print('peername:', clisock.getpeername())
    data = clisock.recv(bufsize) # receive request
    if not data:
        continue
    fin = open('message.txt',r) # read the angle info from the file
    msg = fin.readline() # read the top line
    fin.close()
    clisock.send(msg.encode(encoding='utf_8', errors='strict'))
    clisock.close() # close this connection and waiting for next one
    print('respond request at time: %s'% (ctime()) )
sock.close()
