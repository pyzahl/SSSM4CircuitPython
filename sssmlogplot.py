import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import sys

print("SSSM Plot. Usage: python sssmlogplot.py 2023xxxxx.log")

logfile = 'sssm.log'

if __name__ == "__main__":
    #print(f"Arguments count: {len(sys.argv)}")
    for i, arg in enumerate(sys.argv):
        #print (arg)
        logfile = arg
        
print ('Reading Logfile: ', logfile)
        
with open(logfile) as f:
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

#fig, axs = plt.subplots(2, 1)

ax1 = plt.subplot(211)
ax2 = plt.subplot(212, sharex=ax1)
axs = [ax1, ax2]

axs[0].plot(t, S)
axs[0].plot(t, Sn)
plt.setp(axs[0].get_xticklabels(), visible=False)

axs[1].set_xlabel('Time in h since '+start)
axs[0].set_ylabel('Seeing in arcs')
axs[0].grid(True)

axs[1].plot(t, I)
axs[1].set_ylabel('Intensity in V')
axs[1].grid(True)

#fig.tight_layout()
plt.show()
