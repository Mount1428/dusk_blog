---
title: "IMU算法：误差状态卡尔曼滤波器"
slug: "IMU-ESKF"
published: 2026-03-02T05:34:12+08:00
summary: "基于误差状态卡尔曼滤波器实现IMU的姿态估计"
cover:
  image: 文章封面图。也支持HTTPS
tags: [IMU, Kalman Filter, ESKF]
categories: '算法'
draft: false
---

## 前置知识

### IMU（惯性测量单元）

IMU是一种传感器，通常包含加速度计和陀螺仪，用于测量物体的加速度和角速度。IMU广泛应用于导航、机器人、无人机等领域。通过处理IMU数据，可以估计物体的姿态（即其在空间中的方向）和位置。然而，由于IMU数据存在噪声和漂移，直接使用原始数据进行姿态估计可能会导致不准确的结果。因此，需要使用滤波算法来处理IMU数据，以获得更准确的姿态估计。

### 欧拉角与四元数

欧拉角是一种表示物体姿态的方式，使用三个角度（滚转、俯仰、偏航）来描述物体在空间中的方向。然而，欧拉角存在万向锁问题，即当某些角度达到特定值时，系统会失去一个自由度，导致姿态估计不稳定。

四元数是一种更稳定的姿态表示方式，由四个分量组成，可以避免万向锁问题，并且在计算上更高效。四元数的乘法可以用于组合旋转，而四元数的共轭可以用于计算逆旋转。因此，在IMU数据处理和姿态估计中，通常使用四元数来表示物体的姿态。

四元数的定义如下：

$$q = w + xi + yj + zk$$

四元数的乘法定义如下：

$$q_1 q_2 = (w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2) + (w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2)i + (w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2)j + (w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2)k$$

四元数的共轭定义如下：

$$q^* = w - xi - yj - zk$$

四元数转换为旋转矩阵的公式如下：

$$R = \begin{bmatrix} 1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\ 2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\ 2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2) \end{bmatrix}$$

### 角速度积分

角速度积分是通过对陀螺仪测量的角速度进行积分来估计物体的姿态。连续时间内，物体的姿态可以通过以下微分方程进行描述（涉及李群代数，不展开描述）：

$$\dot{R}(t) = R(t) \hat{\omega}(t)$$

其中，$R(t)$表示物体在时间$t$时刻的旋转矩阵，$\hat{\omega}(t)$表示由陀螺仪测量的角速度构成的反对称矩阵。通过对上述微分方程进行积分，可以得到物体在时间$t+\Delta t$时刻的旋转矩阵。角速度的反对称矩阵定义如下：

$$\hat{\omega} = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}$$

使用四元数形式的姿态更新公式如下：

$$\dot{q}(t) = \frac{1}{2} q(t) \otimes \omega(t)$$

假设在时间$t$时刻，陀螺仪测量的角速度为$\omega(t)$，则在时间$t+\Delta t$时刻，物体的姿态可以通过以下公式进行更新：

$$q(t+\Delta t) = q(t) \otimes \exp\left(\frac{1}{2} \omega(t) \Delta t\right)$$

其中，$\exp\left(\frac{1}{2} \omega(t) \Delta t\right)$表示将角速度$\omega(t)$转换为四元数的指数映射。在间隔$\Delta t$较小的情况下，可以近似为：

$$\exp\left(\frac{1}{2} \omega(t) \Delta t\right) \approx \begin{bmatrix} \cos\left(\frac{|\omega(t)| \Delta t}{2}\right) \\ \frac{\omega(t)}{|\omega(t)|} \sin\left(\frac{|\omega(t)| \Delta t}{2}\right) \end{bmatrix}$$

### Kalman Filter（卡尔曼滤波器）

卡尔曼滤波器是一种递归的最优估计算法，适用于线性系统。它通过预测和更新两个步骤来估计系统状态。在预测步骤中，卡尔曼滤波器使用系统的动态模型来预测下一个状态和误差协方差。在更新步骤中，卡尔曼滤波器使用新的测量数据来修正预测的状态和误差协方差。基本公式如下：

预测步骤：

$$\hat{x}_{k|k-1} = F \hat{x}_{k-1|k-1} + B u_k$$

$$P_{k|k-1} = F P_{k-1|k-1} F^T + Q$$

更新步骤：

$$K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}$$
$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})$$
$$P_{k|k} = (I - K_k H) P_{k|k-1}$$

## 原理与思路

ESKF（Error State Kalman Filter）是一种基于误差状态的卡尔曼滤波器，适用于非线性系统的状态估计。ESKF的核心思想是将系统状态分为两个部分：名义状态和误差状态。名义状态表示系统的实际状态，而误差状态表示名义状态与真实状态之间的误差。通过对误差状态进行卡尔曼滤波，可以更好地处理非线性系统的估计问题。

### 区别KF、EKF和ESKF

- KF（Kalman Filter）适用于线性系统，假设系统的状态转移和测量模型都是线性的。
- EKF（Extended Kalman Filter）适用于非线性系统，通过局部线性化系统模型来进行状态估计，但可能会引入较大的线性化误差。
- ESKF（Error State Kalman Filter）通过引入误差状态来处理非线性系统的状态估计问题，将线性化约束在原点附近，从而减少线性化误差，提高估计的准确性和稳定性。

### 原理

ESKF整体流程如下：当IMU测量数据到达时，我们把它积分后，放入名义状态变量中。由于这种做法没有考虑噪声，其结果自然会快速漂移，于是我们希望把误差部分作为误差变量，放在ESKF中。ESKF内部会考虑各种噪声和零偏的影响，并且给出误差状态的一个高斯分布描述。同时，ESKF本身作为一种卡尔曼滤波器，也具有预测过程和修正过程，其中修正过程需要依赖IMU以外的传感器观测。当然，在修正之后，ESKF可以给出后验的误差高斯分布，随后我们可以把这部分误差放入名义状态变量中，并把ESKF置零，这样就完成了一次循环。

ESKF本身可以融合多个数据源估计位置、速度、姿态等状态信息。考虑到后续仅使用IMU数据进行姿态估计，故此处仅估计姿态和角速度偏置两个状态。

名义状态（nominal state）向量定义为：

$$x = \begin{bmatrix} q \\ b_g \end{bmatrix}$$

其中，$q$表示姿态四元数，$b_g$表示陀螺仪的三轴角速度偏置。

误差状态（error state）向量定义为：

$$\delta x = \begin{bmatrix} \delta \theta \\ \delta b_g \end{bmatrix}$$

其中，$\delta \theta$表示姿态误差，$\delta b_g$表示陀螺仪三轴角速度偏置误差。

#### 预测过程

误差状态变量 $\delta x$ 的离散时间运动方程写作线性化形式：

$$\delta x_{k+1} = F_k \delta x_k + w_k,\quad w_k\sim\mathcal{N}(0,Q_d)$$

这里采用常见符号与约定：

- 误差态：$\delta x = \begin{bmatrix}\delta\theta\\ \delta b_g\end{bmatrix} ^ T$，其中 $\delta\theta$ 为小角误差（向量形式）。
- 反对称矩阵（叉乘矩阵）记为 $[v]_{\times}$。

线性化离散转移矩阵常用近似（在 $\Delta t$ 小的情况下）为：

$$F_k \approx
\begin{bmatrix}
R((\omega_{meas}-b_g) \Delta t)^T & -I\Delta t\\[4pt]
0 & I
\end{bmatrix}$$

说明与约定：
- 该式中 $\omega_{meas}$ 为陀螺仪测量值，$b_g$ 为名义偏置。
- 过程噪声离散协方差建议按物理来源区分——陀螺仪测量白噪声与偏置随机游走。

常用的离散过程噪声结构（示例）：

$$Q_d = \begin{bmatrix}
\sigma_\omega^2 \, I_3 \,\Delta t & 0\\[4pt]
0 & \sigma_b^2 \, I_3 \,\Delta t
\end{bmatrix}$$

其中 $\sigma_\omega^2$ 是角速率白噪声强度，$\sigma_b^2$ 是偏置随机游走强度（连续到离散的近似）。

名义态（nominal state）与误差态的预测分开进行：

1. 名义态（非线性）积分：
   - 角增量： $\Delta \theta = (\omega_{meas} - b_g)\Delta t$；
   - 四元数更新（右乘约定）：
     $$\delta q(\Delta \theta) = \exp\!\left(\tfrac{1}{2}\Delta \theta\right)
     = \begin{bmatrix}\cos\frac{\|\Delta \theta\|}{2}\\[2pt]
     \dfrac{\Delta \theta}{\|\Delta \theta\|}\sin\frac{\|\Delta \theta\|}{2}\end{bmatrix},\quad
     q \leftarrow q \otimes \delta q(\Delta \theta).$$
   - 对于小角近似：$\delta q \approx [1,\tfrac{1}{2}\delta\theta]^T$（四元数标量在前）。
   - 偏置名义态常保持积分或视为随机游走： $b_g \leftarrow b_g$（或按偏置模型更新）。

2. 误差态预测（线性）：
   $$\hat{\delta x}_{k|k-1} = F_k \hat{\delta x}_{k-1|k-1}$$
   $$P_{k|k-1} = F_k P_{k-1|k-1} F_k^\top + Q_d$$

注意：由于每次测量后误差态会被注入并重置为零，$\hat{\delta x}_{k|k-1}$ 通常为零；但协方差 $P$ 的预测仍需计算。

#### 更新过程

ESKF 的更新针对误差态进行（观测模型线性化在误差原点附近）：

- 典型观测（例如利用加速度计观测重力方向）：
  - 加速度计在静止或匀速下观测值（body frame）并一阶近似展开：
    $$\begin{aligned} z &= R(q_{true})^\top g_w  \\ &= ({R(q_{nom})(I + [\delta \theta]_{\times})}) ^T g_w  \\ &= {R(q_{nom})^T g_w + [R(q_{nom})^T g_w]_{\times} \delta \theta} \end{aligned}$$
  - 线性化得到观测矩阵 $H$：
    $$H = \begin{bmatrix}[R(q_{nom})^\top g_w]_\times & 0_{3\times3}\end{bmatrix}$$

更新步骤（误差态）：
1. 计算残差 $\tilde y = z - h(q)$（非线性观测模型的残差）。
2. 计算增益：
   $$K = P_{k|k-1} H^\top (H P_{k|k-1} H^\top + R)^{-1}.$$
3. 误差态后验：
   $$\delta\hat x_{k|k} = K \tilde y.$$
4. 协方差更新（可用 Joseph 形式以保数值稳定）：
   $$P_{k|k} = (I-KH)P_{k|k-1}(I-KH)^\top + K R K^\top.$$

注入与重置（inject & reset）：
- 将误差注入到名义态（右乘约定）：
  - 取 $\delta\theta$（前三分量），构造小角四元数 $\delta q(\delta\theta) \approx \exp\left(\tfrac{1}{2}\delta \theta\right)$；
  - 名义四元数更新： $q \leftarrow q \otimes \delta q(\delta\theta)$ 并归一化；
  - 偏置注入： $b_g \leftarrow b_g + \delta b_g$。
- 将误差态重置为零： $\hat{\delta x}\leftarrow 0$。
- 协方差相应调整（已经通过 $(I-KH)P$ 或 Joseph 形式处理）。

考虑到仅IMU观测下无法对Yaw角进行约束，故对于误差状态的注入过程中，Yaw误差分量不进行注入（即保持为零，包括角度误差和陀螺仪偏置误差）。

#### 卡方校验

卡方校验用于评估观测残差是否符合预期的统计分布，从而判断滤波器的性能和数据的可靠性。对于一个具有 $n$ 个观测维度的系统，残差 $\tilde y$ 的卡方统计量计算如下：

$$\chi^2 = \tilde y^\top (H P_{k|k-1} H^\top + R)^{-1} \tilde y$$

其中，$H P_{k|k-1} H^\top + R$ 是残差的协方差矩阵。根据卡方分布的性质，如果 $\chi^2$ 的值超过了某个置信水平对应的临界值，则可以认为观测数据存在异常，可能是由于传感器故障、环境干扰等原因引起的。

对于IMU加速度计观测重力方向的情况，通常使用3维残差，因此卡方分布的自由度为3。根据所选的置信水平（例如95%），可以查找对应的临界值来进行判断。如判断为异常，可能需要丢弃当前观测数据或调整滤波器的参数以提高鲁棒性。

特别指出的是，被检测物体稳定的情况下，若长时间卡方校验结果异常，可能表明滤波器发散，此时考虑进行强制更新（例如强制进行观测更新、重置协方差或调整过程噪声）以恢复滤波器的稳定性。

## 结果与分析

测试平台：达妙MC-02（STM32H723VGT6 500MHz），启用ICache和DCache。

![静止状态Roll](https://cdn.mountdusk.top/img/%E9%9D%99%E6%AD%A2%E7%8A%B6%E6%80%81Roll.png)  ![静止状态Pitch](https://cdn.mountdusk.top/img/%E9%9D%99%E6%AD%A2%E7%8A%B6%E6%80%81Pitch.png)  ![静止状态Yaw](https://cdn.mountdusk.top/img/%E9%9D%99%E6%AD%A2%E7%8A%B6%E6%80%81Yaw.png)

{{< video src="https://cdn.mountdusk.top/img/%E9%9D%99%E6%AD%A2%E7%8A%B6%E6%80%81%E5%BD%95%E5%B1%8F.webm" >}}

静止状态下Roll轴和Pitch轴的变化区间在0.7°左右，Yaw轴漂移在短时间内基本注意不到。

![静止1小时后Yaw](https://cdn.mountdusk.top/img/%E9%9D%99%E6%AD%A21%E5%B0%8F%E6%97%B6%E5%90%8E%E8%AF%BB%E6%95%B0.png)

Yaw轴零飘在27°/hour左右（0.45°/min），对于RM场景已经足够使用。增加温控后理论上可以更低。

![典型计算时间](https://cdn.mountdusk.top/img/%E8%AE%A1%E7%AE%97%E5%BB%B6%E8%BF%9F.png)

仅预测过程一次计算大约90us，预测+更新一次计算190us左右。考虑到H7和Cache的加成，这个延迟表现并不算好，应该是笔者本人在矩阵计算等方面的设计较差导致（使用的是笔者自己写的矩阵库，使用C++表达式模板，有很多的短函数调用）。使用Release模式编译后预测+更新一次计算大约为30us。

由于笔者没有方便测试的设备，对于运动状态下的效果只能通过手动甩动IMU测得。

{{< video src="https://cdn.mountdusk.top/img/%E5%BF%AB%E9%80%9F%E7%94%A9%E5%8A%A8%E5%BD%95%E5%B1%8F.webm" >}}

![快速甩动后收敛](https://cdn.mountdusk.top/img/%E5%BF%AB%E9%80%9F%E7%94%A9%E5%8A%A8%E5%90%8E%E6%94%B6%E6%95%9B.png)

由于笔者并没有给很严格的卡方门限，运动过程中的动态加速度的分离效果还是较差，但同时也获得了很快的静止后姿态收敛（50ms以内）。

<!-- ## 未测试的优化方向

### 自适应R噪声矩阵

### 优化单子样补偿算法（圆锥误差补偿） -->

---

## 参考文章

- [简明ESKF推导——知乎](https://zhuanlan.zhihu.com/p/441182819)
- [IMU算法：终 | 算法设计实践：采用ESKF的解算——搬砖笔记](https://sourcelizi.github.io/202311/imu-practice3/)
