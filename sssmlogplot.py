import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

print("SSSM Plot")

with open("sssm.log") as f:
    lines = f.readlines()
    t  = np.array([line.split(';')[0] for line in lines], dtype=float)/1000
    S  = np.array([line.split(';')[2] for line in lines], dtype=float)
    I  = np.array([line.split(';')[3] for line in lines], dtype=float)
    Sn = np.array([line.split(';')[4] for line in lines], dtype=float)

t0 = t[0]
t = t-t0
t = t/3600
    
# 1676741308549;2023-02-18T17:28:28 UTC;  3.17;  0.59;  5.44;

datetime.fromtimestamp(t0).strftime("%I:%M:%S")
start = datetime.fromtimestamp(t0).strftime("%A, %B %d, %Y %I:%M:%S")
#'Sunday, January 29, 2017 08:30:00'


fig, axs = plt.subplots(2, 1)
axs[0].plot(t, S)
axs[0].plot(t, Sn)

axs[0].set_xlabel('Time in h since '+start)
axs[0].set_ylabel('Seeing in arcs')
axs[0].grid(True)

axs[1].plot(t, I)
axs[1].set_ylabel('Intensity in V')

fig.tight_layout()
plt.show()
