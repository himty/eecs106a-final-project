#!/usr/bin/env python
import fileinput
import time

while True:
	for line in fileinput.input(r'message.txt', inplace=1):
		if not fileinput.isfirstline():
			print(line.replace('\n',''))

	time.sleep(50/1000) # make sure data will be updated in 50ms
