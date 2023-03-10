import numpy as np
import matplotlib.pyplot as plt

zeta = 1.0
zeta2 = 1.0
d_ro = 1.0
obs_effect_r = 0.5

r = [i/100.0 for i in range(30, 100)]
f = []
ft = []
for rr in r:
    if rr>obs_effect_r:
        ff=0
    else:
        ff = (zeta*1)*(((1/rr)-(1/obs_effect_r))**2)*(1/rr)**2
    f.append(ff)
    ft.append(zeta2*rr)

 
plt.plot(r, f, '-b')
plt.plot(r, ft, '-r')

plt.show()