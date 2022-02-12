# -*- coding: utf-8 -*-
"""
Created on Mon Jun  4 03:29:52 2018
Author: Yuchu He
E-mail: heyuchu@foxmail.com
"""
import math
import numpy as np
import matplotlib.pyplot as plt

#参数定义
NUM = 10
TIME = 1000
LENGTH = 800
DINIT = 15
DSAFE = 7
TSAFE = 1.5
VINIT = 8
VMAX = 15
VMIN = 0
AINIT = 0
AMAX = 2
AMIN = -4
STEP = 0.1

x = np.zeros((NUM, TIME))
v = np.zeros((NUM, TIME))
a = np.zeros((NUM, TIME))

#初始化车辆数据
def init_veh():
    for i in range(NUM):
        x[i][0] = (NUM - i - 1) * DINIT
        v[i][0] = VINIT
        a[i][0] = AINIT

#更新车辆数据
def update_veh(i, t):
    a[i][t] = idm_acc(i, t)
    v[i][t] = v[i][t-1] + a[i][t] * STEP
    x[i][t] = x[i][t-1] + v[i][t-1] * STEP + 0.5 * a[i][t] * STEP * STEP
    if a[i][t] > AMAX:
        a[i][t] = AMAX
    if a[i][t] < AMIN:
        a[i][t] = AMIN
    if v[i][t] > VMAX:
        v[i][t] = VMAX
    if v[i][t] < VMIN:
        v[i][t] = VMIN

#使用idm模型计算加速度
def idm_acc(i, t):
    vt = v[i][t-1]
    vmax = VMAX
    s0 = DSAFE
    T = TSAFE
    alpha = AMAX
    beta = -AMIN
    if i==0:
        dx = 99999
        dv = vt
    else:
        dx = x[i-1][t-1] - x[i][t-1]
        dv = v[i][t-1] - v[i-1][t-1]
    
    sn = s0 + vt * T + vt * dv / (2 * math.sqrt(alpha * beta))
    result = alpha * (1 - pow(vt/vmax, 4) - pow(sn/dx, 2))
    return result

#绘图：位置、速度、加速度
def plot_result():
    fig = plt.figure()
    fig_x = fig.add_subplot(2, 2, 1)
    fig_v = fig.add_subplot(2, 2, 2)
    fig_a = fig.add_subplot(2, 2, 3)
    i = range(TIME)
    for n in range(NUM):
        fig_x.plot(i, x[n])
        fig_v.plot(i, v[n])
        fig_a.plot(i, a[n])

    fig_x.set_xlabel('$time(s)$')
    fig_x.set_ylabel('$space(m)$')
    
    fig_v.set_xlabel('$time(s)$')
    fig_v.set_ylabel('$velocity(m/s)$')
  
    fig_a.set_xlabel('$time(s)$')
    fig_a.set_ylabel('$acceleration(m/s2)$')
    
    fig.show()
    plt.show()

#主函数，从这里开始执行
def simulate():
    init_veh()

    for t in np.arange(1, TIME):
        for i in range(NUM):
            update_veh(i, t)
    
    plot_result()

simulate()
