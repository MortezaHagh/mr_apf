import numpy as np
from smallest_circle import *
import matplotlib.pyplot as plt

thetas = np.linspace(0, np.pi*2, 100)
a=[1,2]
b = [4,5]
p = [a, b]

c = make_circle(p)
print(c)

x = [pp[0] for pp in p]
y = [pp[1] for pp in p]

xr = [c[0]+c[2]*np.cos(th) for th in thetas]
yr = [c[1]+c[2]*np.sin(th) for th in thetas]

plt.plot(x, y, 'bo')
plt.plot(xr, yr)


plt.show()