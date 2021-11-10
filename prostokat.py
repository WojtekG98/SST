import matplotlib.pyplot as plt
import random

w = random.randrange(5, 20)
N = 100
def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if N/2+w+1 >= x >= N/2-w-1 and N/2+w+1 >= y >= N/2-w-1:
        return False
    else:
        return True

def isStateValid2(state):
    x = state[0]
    y = state[1]
    if N/2+w+1 >= x >= N/2-w-1 and N/2+w+1 >= y >= N/2-w-1:
        return False
    else:
        return True

def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    rec1 = plt.Rectangle([N/2-w, N/2-w], 2*w, 2*w, color='k')
    plt.gcf().gca().add_artist(rec1)


if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
