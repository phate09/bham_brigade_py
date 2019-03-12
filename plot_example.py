import numpy as np
import matplotlib.pyplot as plt

# plt.axis([0, 10, 0, 1])


import numpy as np
from matplotlib import pyplot as plt
def main3():
    a = np.random.random((16, 16))
    graph=plt.imshow(a, cmap='hot', interpolation='nearest')
    plt.show(block=False)
    for i in range(1000):
        graph.set_data(np.random.random((1000, 1000)))
        plt.pause(0.5)
    plt.show()
def main():
    plt.axis([-50,50,0,10000])
    plt.ion()
    plt.show()

    x = np.arange(-50, 51)
    for pow in range(1,50):   # plot x^1, x^2, ..., x^4
        y = [Xi**pow for Xi in x]
        plt.plot(x, y)
        plt.draw()
        plt.pause(0.001)
        input("Press [enter] to continue.")

if __name__ == '__main__':
    main3()

def main2():
    for i in range(10000):
        y = np.random.random()
        plt.scatter(i, y)
        plt.pause(0.05)

    plt.show()