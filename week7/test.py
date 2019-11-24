#! /usr/bin/python3
import numpy as np
import matplotlib as mt
import matplotlib.pyplot as plt

fnam = 's1.nptxt'

x, y = np.loadtxt(fnam,dtype=float)

# print(f'ANGLES:{x}')
# print(f'READINGS:{y}')

plt.axis([0,255,-180,180])
plt.xticks(np.arange(-180, 180+1, 60))
# plt.xlim(-180,180)


fig = plt.figure()
ax = plt.subplot(111)
plt.grid(color='blue',linestyle='-',linewidth=0.5)
# ax.scatter(x,y,alpha=0.5, marker='x', s=10)
plt.plot(x,y)
fig.savefig('temp.png')