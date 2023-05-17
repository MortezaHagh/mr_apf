import numpy as np

a1 = 10*np.pi/180
a2 = -10*np.pi/180

a_diff = a2 - a1
a_diff = np.arctan2(np.sin(a_diff), np.cos(a_diff))

print(a_diff, a_diff*180/np.pi)