# import numpy as np
# from itertools import count
# import random
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
#
# x_vals = []
# y_vals = []
#
# plt.plot(x_vals, y_vals)
#
# index = count()
#
#
# def animate(i):
#     x_vals.append(next(index))
#     y_vals.append(random.randint(0, 5))
#
#     plt.cla()
#     plt.plot(x_vals, y_vals)
#
#
# ani = FuncAnimation(plt.gcf(), animate, interval=1000)
#
# plt.tight_layout()
# plt.show()

from vis import *
data = MyDataClass()
plotter = MyPlotClass(data)
fetcher = MyDataFetchClass(data)

fetcher.start()
plt.show()
fetcher.join()
