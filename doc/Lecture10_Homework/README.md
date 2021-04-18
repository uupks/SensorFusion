# 第十章作业

基于图优化的滑动窗模型包含：
* 地图匹配位姿和优化变量的残差
* 激光里程计相对位姿和优化变量的残差
* IMU预积分和优化变量的残差
* 边缘化形成的先验因子对应的残差

所以首先需要推导各残差关于优化变量的雅可比

## 1. 推导各残差关于优化变量的雅可比

### 地图匹配位姿

* Residual
  <!-- $$
  e_p = t - t_{obs}
  $$ -->
  ![e_p](https://g.yuque.com/gr/latex?e_p%20%3D%20t%20-%20t_%7Bobs%7D%0A#card=math&code=e_p%20%3D%20t%20-%20t_%7Bobs%7D%0A)

  <!-- $$
  e_{lie} = \ln(R_{obs}^{T}R)^{\vee} = ln(\exp(\phi_{1}^{\hat{}}))^{\vee}
  $$ -->
  ![e_lie](https://g.yuque.com/gr/latex?e_%7Blie%7D%20%3D%20%5Cln(R_%7Bobs%7D%5E%7BT%7DR)%5E%7B%5Cvee%7D%20%3D%20ln(%5Cexp(%5Cphi_%7B1%7D%5E%7B%5Cvee%7D))%0A#card=math&code=e_%7Blie%7D%20%3D%20%5Cln%28R_%7Bobs%7D%5E%7BT%7DR%29%5E%7B%5Cvee%7D%20%3D%20ln%28%5Cexp%28%5Cphi_%7B1%7D%5E%7B%5Cvee%7D%29%29%0A)

* Jacobian
  <!-- $$
  \frac{\partial{e_{p}}}{\partial{t}} = I_3
  $$ -->
  ![J_dep_dt](https://g.yuque.com/gr/latex?%5Cfrac%7B%5Cpartial%7Be_%7Bp%7D%7D%7D%7B%5Cpartial%7Bt%7D%7D%20%3D%20I_3%0A#card=math&code=%5Cfrac%7B%5Cpartial%7Be_%7Bp%7D%7D%7D%7B%5Cpartial%7Bt%7D%7D%20%3D%20I_3%0A)

  <!-- $$
  \begin{aligned}
  \frac{\partial{e_{lie}}}{\partial{\delta{\phi}}} 
  & = \frac{\partial\ln(R_{obs}^{T}R\exp{(\delta\phi^{\hat{}})})^{\vee}}{\partial{\delta{\phi}}} \\
  & = \frac{\partial\ln{(\exp{(\phi_{1}^{\hat{}})}\exp{(\delta{\phi^{\hat{}}})})^{\vee}}}{\partial{\delta{\phi}}} \\
  & \approx \frac{\partial(({\phi_{1}} + J_r(\phi_1)^{-1})\cdot{\delta\phi})}{\partial{\delta\phi}} \\
  & = J_r(\phi_1)^{-1}
  \end{aligned}
  $$ -->

  ![](https://cdn.nlark.com/yuque/__latex/58dfc6b79307a2244774ce6b00b87503.svg#card=math&code=%20%20%5Cbegin%7Baligned%7D%0A%20%20%5Cfrac%7B%5Cpartial%7Be_%7Blie%7D%7D%7D%7B%5Cpartial%7B%5Cdelta%7B%5Cphi%7D%7D%7D%20%0A%20%20%26%20%3D%20%5Cfrac%7B%5Cpartial%5Cln%28R_%7Bobs%7D%5E%7BT%7DR%5Cexp%7B%28%5Cdelta%5Cphi%5E%7B%5Chat%7B%7D%7D%29%7D%29%5E%7B%5Cvee%7D%7D%7B%5Cpartial%7B%5Cdelta%7B%5Cphi%7D%7D%7D%20%5C%5C%0A%20%20%26%20%3D%20%5Cfrac%7B%5Cpartial%5Cln%7B%28%5Cexp%7B%28%5Cphi_%7B1%7D%5E%7B%5Chat%7B%7D%7D%29%7D%5Cexp%7B%28%5Cdelta%7B%5Cphi%5E%7B%5Chat%7B%7D%7D%7D%29%7D%29%5E%7B%5Cvee%7D%7D%7D%7B%5Cpartial%7B%5Cdelta%7B%5Cphi%7D%7D%7D%20%5C%5C%0A%20%20%26%20%5Capprox%20%5Cfrac%7B%5Cpartial%28%7B%5Cphi_%7B1%7D%7D%20%2B%20J_r%28%5Cphi_1%29%5E%7B-1%7D%5Ccdot%7B%5Cdelta%5Cphi%7D%29%7D%7B%5Cpartial%7B%5Cdelta%5Cphi%7D%7D%20%5C%5C%0A%20%20%26%20%3D%20J_r%28%5Cphi_1%29%5E%7B-1%7D%0A%20%20%5Cend%7Baligned%7D&height=151&width=225)

### 激光里程计相对位姿
* Residual
  
  $$
  e_p = R_i^T(t_j-t_i) - t_{obs}
  $$

  $$
  e_{lie} = ln(R_{ij\_obs}^T\cdot(R_i^T\cdot R_j))^{\vee}
  $$
* Jacobian
  
  $$
  \begin{aligned}
    \frac{\partial e_p}{\partial{t_i}} = -R_i^T
  \end{aligned}
  $$

  $$
    \frac{\partial{e_p}}{\partial{t_j}} = R_i^T
  $$

  $$
  \begin{aligned}
    \frac{\partial{e_{lie}}}{\partial{R_i}} = 
  \end{aligned}
  $$

  $$
  \begin{aligned}
    \frac{\partial{e_{lie}}}{\partial{R_j}} = 
  \end{aligned}
  $$

### IMU预积分



### 边缘化形成的先验因子

## 2. 基于图优化的定位

作业是**参考了葛垚大佬的推导**，先完成了代码补全，运行起来看了一下效果

![running1](./running1.png)
![running2](./running2.png)

|  | Optimized | LidarOdometry |
|:----:   | :-----: | :-----: |
| Map | ![map](./opt_map.png) | ![raw](./laser_map.png) |

| APE  | Optimized | LidarOdometry |
|:----:   | :-----: | :-----: |
| max | 8.442891 | 7.678134 |
| mean | 4.747836 | 4.170015 |
| median | 4.815285 | 4.252581 |
| min | 0.000002 | 0.000002 |
| rmse | 5.008378 | 4.514122 |
| sse | 113554.602350 | 92248.011402 |
| std | 1.594337 | 1.728660 |

