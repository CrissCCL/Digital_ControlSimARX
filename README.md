# 🧪 Digital Control Simulation — ARX Model + Saturation

This repository provides a **tutorial-oriented simulation** of a **digital PI control loop** applied to a **discrete-time ARX model** identified from experimental data.  
The objective is to reproduce in simulation the **same discrete behavior** expected when the controller is later implemented on a **microcontroller** (Arduino, Teensy, ESP32, etc.), including **actuator saturation**.

> ⚠️ **Note:** This tutorial is for **educational purposes only**. It focuses on simulation and understanding discrete-time digital control with ARX models.

## 🎯 Goals of the Tutorial

- Model a system using a **discrete-time ARX model** identified experimentally  
- Implement a **discrete PI controller** using incremental form  
- Add **saturation limits** to emulate real actuator constraints  
- Compare reference tracking and control signal behavior  

## 🧩 ARX System Model

The discrete-time ARX model is defined as:

$$
A(z)y(k) = B(z)u(k) + e(k)
$$

Where:

$$
A(z) = 1 - 1.937 z^{-1} + 1.152 z^{-2} - 0.2144 z^{-3}
$$

$$
B(z) = -0.001961 z^{-3}
$$
```matlab
a = [1  -1.9366 1.1523 -0.2144]; % note the sign for implementation
b = [0 0 0 -0.001961];       % B(z) with zeros for delays
```
The discrete model implemented is:

```matlab
y(k) = -a(2)*y1 - a(3)*y2 - a(4)*y3  + b(1)*u1 + b(2)*u2 + b(3)*u3+ b(4)*u4;
```

- Sampling period: $$T_s = 0.004 s$$  
- This model was identified from experimental data using **ARX method**.

## ⚙️ Digital PI Controller (Incremental Form)

The discrete control law implemented is:

$$
u(k) = u(k-1) + K_0 e(k) + K_1 e(k-1)
$$

```matlab
error  = Ref(k) - y(k);
u  = u1 + K0*error + K1*error1;
```

With tuning parameters derived from:
- Proportional gain: $$K_p$$
- Integral time: $$T_i$$
- Sampling time: $$T_s$$

Where

$$
K_0 = K_p + \frac{K_p}{2T_i}T_s
$$

$$
K_1 = -K_p + \frac{K_p}{2T_i}T_s
$$


## 🔒 Actuator Saturation
To emulate microcontroller behavior, the controller output is **limited to a predefined range**:

- Prevents unrealistic actuator commands  
- Reflects PWM or DAC limits on embedded hardware  
- Avoids integrator wind-up if actuator saturates
- 
Example: -100% ≤ u(n) ≤ 100%

Without saturation, simulation results may falsely assume an ideal actuator with infinite authority, which never matches microcontroller deployments.
To emulate real microcontroller behavior — such as PWM range or fixed DAC limits — 

```matlab
if u > 100
    u=100;
end
if u <-100
    u=-100;
end
```
Below are example plots generated with the script:

<p align="center">
<img width="500" alt="Control simulation" src="https://github.com/user-attachments/assets/15f5a4c5-274d-4731-95b4-0aaa68f71ed3" />
</p>




## 📜 License
MIT License
