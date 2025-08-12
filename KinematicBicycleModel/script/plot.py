import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    """
    Description:
    This script visualizes the results of a Stanley Controller simulation.
    
    Input:
    - input.csv: contains reference trajectory columns (cx, cy)
    - output.csv: contains actual vehicle path and states (x, y, v, theta, t)

    Output:
    - Plots:
      1. Planned trajectory vs. Stanley controller path
      2. Vehicle velocity vs. target velocity over time
      3. Heading angle (theta) vs. time
    """

    df = pd.read_csv("data/input.csv")
    df1 = pd.read_csv("output/output.csv")

    cx = np.array(df.cx)
    cy = np.array(df.cy)
    x = np.array(df1.x)
    y = np.array(df1.y)
    theta = np.array(df1.theta)
    v = np.array(df1.v)
    t = np.array(df1.t)
    tv = np.ones(len(t)) * 10.0

    plt.figure(1)
    plt.plot(cx, cy, label='Reference Trajectory', color='b')
    plt.plot(x, y, label='Stanley Controller Path', color='r')
    plt.title("Trajectory and Stanley Controller Path")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.axis('equal')

    plt.figure(2)
    plt.plot(t, v, label='Actual Velocity')
    plt.plot(t, tv, label='Target Velocity', linestyle='--')
    plt.title("Velocity vs Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.legend()

    plt.figure(3)
    plt.plot(t, theta, label='Heading (Theta)')
    plt.title("Theta vs Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Theta [rad]")
    plt.legend()

    plt.show()

if __name__ == "__main__":
    main()
