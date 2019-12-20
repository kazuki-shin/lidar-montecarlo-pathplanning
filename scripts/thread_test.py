from threading import Lock, Thread
import numpy as np
import time
from plan import display_plan
latest_frame = None

def render():
    while(True):
        if latest_frame is not None:
            img = display_plan(latest_frame)
            print("Done")


thread = Thread(target=render)
thread.start()

while(True):
    mat = np.load('points.npy')[:,[0,1]]
    latest_frame = mat
    time.sleep(1)
