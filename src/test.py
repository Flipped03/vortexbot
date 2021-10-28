import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import csv
from matplotlib.pyplot import MultipleLocator

# ax = plt.axes(projection='3d')

filepath = "/home/li/vortex_ws/src/vortexbot/traj_ref.txt"
filepath1 = "/home/li/vortex_ws/src/vortexbot/real_ref.txt"



with open(filepath, "r") as f:  # 打开文件
    data = f.readline()  # 读取文件
    data = data.strip('\n')
    x= []
    y = []
    while (data):
        data = data.strip('\n')
        a = data.split(',')
        # print(a[0])
        #b = a.read_until(',')
        x.append(a[0])
        y.append(a[1])
        data = f.readline()

with open(filepath1, "r") as ff:  # 打开文件
    data = ff.readline()  # 读取文件
    data = data.strip('\n')
    x1= []
    y1 = []
    while (data):
        data = data.strip('\n')
        a = data.split(',')
        # print(a[0])
        #b = a.read_until(',')
        x1.append(a[0])
        y1.append(a[1])
        data = ff.readline()

x = list(float(i) for i in x)
y = list(float(i) for i in y)
x1 = list(float(i) for i in x1)
y1 = list(float(i) for i in y1)


plt.axis('equal')
plt.plot(x,y,marker='.', color='green')
plt.plot(x1,y1,marker='.', color='red')
# # # plt.plot(smooth_x,smooth_y,marker='.', color='red')
#plt.plot(x_t,speed[0:len(t)],marker='.', color='green')
# plt.plot(t,smooth_speed1[0:num],marker='.', color='blue')
# ax.set_xlim3d(smooth_x[0],smooth_x[-1])   #指定x轴坐标值范围
# ax.set_ylim3d(smooth_x[0],smooth_x[-1])   #指定y轴坐标值范围
# # ax.daspect([1,1,10])
# ax.axis('auto')
# ax.plot3D(x, y,speed, marker='.', color='blue')
# ax.plot3D(smooth_x1, smooth_y1,smooth_speed1, marker='.', color='red')
plt.show()

# print(x)
# print(y)
