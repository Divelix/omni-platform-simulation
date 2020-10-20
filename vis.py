import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class OmniPlotter:
    def __init__(self, car):
        self._car = car
        # self.line1, = plt.plot(0, 0, color="r", label="ground truth")
        self.line2, = plt.plot(0, 0, color="b", label="estimated value")
        plt.legend(loc='upper right')
        self.anim = FuncAnimation(plt.gcf(), self.update, interval=1000, repeat=True)

    def update(self, i):
        # self.line1.set_data(self._car.x_real, self._car.y_real)
        self.line2.set_data(self._car.x_estm, self._car.y_estm)
        self.line2.axes.relim()
        self.line2.axes.autoscale_view()
