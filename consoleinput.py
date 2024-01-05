import threading

consoleBuffer = []

def consoleInput(myBuffer):
    while True:
        myBuffer.append(input())
 
threading.Thread(target=consoleInput, args=(consoleBuffer,), daemon=True).start() # start the thread

import time # just to demonstrate non blocking parallel processing

while True:
    time.sleep(2) # avoid 100% cpu
    #print(time.time()) # just to demonstrate non blocking parallel processing
    while consoleBuffer:
        print(repr(consoleBuffer.pop(0)))