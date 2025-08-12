import matplotlib.pyplot as plt 
import numpy as np 
import pandas as pd 

df_ = pd.read_csv("InputData/input.csv")


x = np.loadtxt("OutputData/x.txt")
y = np.loadtxt("OutputData/y.txt")
yaw = np.loadtxt("OutputData/yaw.txt")
v = np.loadtxt("OutputData/v.txt")
t = np.loadtxt("OutputData/t.txt")

fx = np.array(df_.cx)
fy = np.array(df_.cy)
fyaw = np.array(df_['cyaw'].to_list()[0:198])
fsp = np.array(df_['sp'].to_list()[0:198])

fig = plt.figure(0)
plt.plot(x, y, label="Controller Trajectory")
plt.plot(fx, fy, label="Reference Path")
plt.legend()
plt.title("Optimal Trajectory")
plt.xlabel("x in m")
plt.ylabel("y in m")

fig2 = plt.figure(1)
plt.plot(t, yaw, label="Controller Yaw")
plt.plot(t, fyaw, label="Reference Yaw")
plt.legend()
plt.title("Yaw")
plt.xlabel("t in s")
plt.ylabel("yaw in radian")

fig3 = plt.figure(2)
plt.plot(t, v, label="Controller Velocity")
plt.plot(t, fsp, label="Planned Velocity")
plt.legend()
plt.title("Velocity")
plt.xlabel("t in s")
plt.ylabel("velocity m/s")

plt.show()