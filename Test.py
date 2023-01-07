import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(-0.05, 1)
ax.set_ylim(-0.05, 1)
plt.grid('on')



#Rotate rectangle patch object
ts = ax.transData
coords = ts.transform([0.2, 0.5])
tr = mpl.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], 90)
t = ts + tr

rec0 = patches.Rectangle((0.2, 0.5), 0.25, 0.2)
ax.add_patch(rec0)

#Rotated rectangle patch
rect1 = patches.Rectangle((0.2, 0.5), 0.25, 0.2, transform=t)
ax.add_patch(rect1)

plt.show()