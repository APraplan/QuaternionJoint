import matplotlib.pyplot as plt
import numpy as np
import time

for i in range(5000):
    plt.plot(np.random.rand(10))
    plt.draw()          # draw current figure
    plt.pause(0.1)      # pause to update the figure, non-blocking
    plt.clf()           # clear the figure for next plot
    time.sleep(0.001)     # simulate some other work in the loop
