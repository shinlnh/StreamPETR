import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from get_waypoint import FILE_REAL_POINT, FILE_PLAN_POINT


def plan_real():
    plan = pd.read_csv(FILE_PLAN_POINT)
    absolute = pd.read_csv(FILE_REAL_POINT)

    plt.clf()
    plt.plot(np.array(plan['x']), np.array(plan['y']), label='plan', color='red')
    plt.scatter(np.array(absolute['x']), np.array(absolute['y']), label='real', color='blue', s=7)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('plan vs real')
    plt.legend()
    plt.savefig('data/plan_vs_real.png')
    

def get_angle():
    data_plan = pd.read_csv(FILE_PLAN_POINT)
    x = np.array(data_plan['x']).reshape((-1, 1))
    y = np.array(data_plan['y']).reshape((-1, 1))
    delta_x = np.vstack([x, x[-1, ]]) - np.vstack([x[0, ], x])
    delta_y = np.vstack([y, y[-1, ]]) - np.vstack([y[0, ], y])
    tan_alpha = delta_y[1:-1] / delta_x[1:-1]

    plt.clf()
    plt.plot(range(len(x) - 1), np.rad2deg(np.arctan(tan_alpha)), label='heading', color='red')
    plt.xlabel('Index')
    plt.ylabel('Angle (deg)')
    plt.title("heading's angle of car")
    plt.legend()
    plt.savefig('data/angle.png')


def main():
    plan_real()
    get_angle()


if __name__ == "__main__":  
    main()
    