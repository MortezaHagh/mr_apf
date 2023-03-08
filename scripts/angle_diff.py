import numpy as np

a1 = np.pi+np.pi/4-np.pi*2
a2 = np.pi+np.pi/3

a_diff = a2 - a1
a_diff = np.arctan2(np.sin(a_diff), np.cos(a_diff))

print(a_diff, a_diff*180/np.pi)