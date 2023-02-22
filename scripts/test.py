import numpy as np

a = 70*np.pi/180
b = 260*np.pi/180
c = 260-70

print(" ================= ")

c = a - b
print("a - b", c*180/np.pi)
cc = np.arctan2(np.sin(c), np.cos(c))
print("cc", cc, cc*180/np.pi)

print("--------")

c = b - a
print("b - a", c*180/np.pi)
cc = np.arctan2(np.sin(c), np.cos(c))
print("cc", cc, cc*180/np.pi)

print(" ================= ")
