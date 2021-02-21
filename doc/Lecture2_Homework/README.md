# 第二章作业

## 1. 跑通提供的工程框架
运行环境为Ubuntu20.04 + ROS Noetic
![Screenshot from 2021-01-26 22-00-59.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1611670870170-d43c40a4-48ad-4fdf-8b0d-4693e2708dac.png#align=left&display=inline&height=1080&margin=%5Bobject%20Object%5D&name=Screenshot%20from%202021-01-26%2022-00-59.png&originHeight=1080&originWidth=1920&size=297871&status=done&style=none&width=1920)
前端ICP点云匹配方法为NDT
![运行截图.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1611672843166-c03d7266-e91b-4d12-af38-42bbb0da4494.png#align=left&display=inline&height=1080&margin=%5Bobject%20Object%5D&name=%E8%BF%90%E8%A1%8C%E6%88%AA%E5%9B%BE.png&originHeight=1080&originWidth=1920&size=274911&status=done&style=none&width=1920)
前端ICP点云匹配方法为ICP
可以看出刚开始 lidar_odometry 和 gnss_odometry 比较接近，随着无人车行驶，lidar_odometry累计误差越来越大，NDT相比ICP误差累计较慢，ICP运行一段时间之后直接跑飞了
## 2. 使用evo计算出分段统计误差和整体轨迹误差
### NDT方法

- 分段误差

![分段误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076395169-82a95dc4-3b7e-44c1-98eb-f4a1ef89e79d.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=83875&status=done&style=none&width=1898)
![分段误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076401105-13cbe3a2-1336-4e1b-bdba-f9e79dbc5ab6.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=221470&status=done&style=none&width=1898)
```
RPE w.r.t. translation part (m)
for delta = 100 (frames) using consecutive pairs
(not aligned)

       max	4.900111
      mean	0.913785
    median	0.702743
       min	0.149742
      rmse	1.180903
       sse	62.753922
       std	0.748016

```

- 整体误差

![整体误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076474354-e87246fc-39c5-45be-838f-d1215d5c65f8.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=72774&status=done&style=none&width=1898)

![整体误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076480710-c2ef88fb-a6b1-44c8-b27b-63d9e0eb9278.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=224491&status=done&style=none&width=1898)
```
APE w.r.t. full transformation (unit-less)
(not aligned)

       max	35.568587
      mean	13.661834
    median	13.013886
       min	0.000001
      rmse	15.913199
       sse	1150423.514716
       std	8.159915
```


### ICP方法

- ICP分段误差

![分段误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076154167-2ba9839b-cd62-49d4-bfac-336b2270e9b1.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=73497&status=done&style=none&width=1898)

![分段误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076160713-654d70a4-7dca-4394-a325-4a367f620d33.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=193236&status=done&style=none&width=1898)
```
RPE w.r.t. translation part (m)
for delta = 100 (frames) using consecutive pairs
(not aligned)

       max	242.260045
      mean	53.016540
    median	26.477208
       min	0.364602
      rmse	81.991107
       sse	302514.373924
       std	62.544289

```

- ICP整体误差

![整体误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076254612-64a41b5a-87b3-49c7-b163-69048e7049f1.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=78685&status=done&style=none&width=1898)

![整体误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076263383-39138d9a-f6d3-49af-85a0-62ca05a0c919.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=189007&status=done&style=none&width=1898)
```
APE w.r.t. full transformation (unit-less)
(not aligned)

       max	1097.214374
      mean	391.479743
    median	333.652511
       min	0.000001
      rmse	506.708539
       sse	1166431348.516783
       std	321.709737

```

## 3. 自己实现点云匹配方法
### 基于Eigen实现的SVD-ICP
![运行截图.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076756812-855fbec2-93fe-410b-a11b-55fc4fdda239.png#align=left&display=inline&height=1080&margin=%5Bobject%20Object%5D&name=%E8%BF%90%E8%A1%8C%E6%88%AA%E5%9B%BE.png&originHeight=1080&originWidth=1920&size=251896&status=done&style=none&width=1920)

- 分段误差

![分段误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076782781-b235d903-f612-44bb-a4cc-744fa2be9b49.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=62058&status=done&style=none&width=1898)

![分段误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076790351-e5d05a93-bc8d-4522-b2f4-1b84d2a166ca.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=204358&status=done&style=none&width=1898)
```
RPE w.r.t. translation part (m)
for delta = 100 (frames) using consecutive pairs
(not aligned)

       max	1098.853797
      mean	177.749155
    median	52.085394
       min	0.410623
      rmse	326.310045
       sse	4791521.060152
       std	273.648467
```

- 整体误差

![整体误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076799332-dcc6dcc5-9d1c-4209-bccc-3e1320066841.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=57883&status=done&style=none&width=1898)

![整体误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612076806204-73e95f64-f5d9-469c-bbe4-c2eb43772917.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=226847&status=done&style=none&width=1898)
```
APE w.r.t. full transformation (unit-less)
(not aligned)

       max	2293.007124
      mean	343.125106
    median	71.965393
       min	0.000001
      rmse	685.504169
       sse	2134828233.712260
       std	593.448505
```
### 基于Eigen实现的GN-ICP

- 运行截图

![运行截图.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612104622210-426d83bc-c8a9-43a4-9398-f4fbd1f7c306.png#align=left&display=inline&height=1080&margin=%5Bobject%20Object%5D&name=%E8%BF%90%E8%A1%8C%E6%88%AA%E5%9B%BE.png&originHeight=1080&originWidth=1920&size=253921&status=done&style=none&width=1920)

- 分段误差

![分段误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612104630694-45fc9374-fa8a-4919-8a22-9bf027c50293.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=70236&status=done&style=none&width=1898)

![分段误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612104637595-229e60f3-3dfb-4e84-a2d2-e58aaf3068d3.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E5%88%86%E6%AE%B5%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=218690&status=done&style=none&width=1898)
```
RPE w.r.t. translation part (m)
for delta = 100 (frames) using consecutive pairs
(not aligned)

       max	97.709731
      mean	22.526664
    median	13.769346
       min	0.126752
      rmse	33.224903
       sse	49675.239105
       std	24.422195
```

- 整体误差

![整体误差.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612104646526-96f6299b-20d2-46f5-a92d-dd555073ff22.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AE.png&originHeight=869&originWidth=1898&size=71727&status=done&style=none&width=1898)

![整体误差map.png](https://cdn.nlark.com/yuque/0/2021/png/2442126/1612104652594-bec8a854-7b20-49ca-864a-22d537135d62.png#align=left&display=inline&height=869&margin=%5Bobject%20Object%5D&name=%E6%95%B4%E4%BD%93%E8%AF%AF%E5%B7%AEmap.png&originHeight=869&originWidth=1898&size=224109&status=done&style=none&width=1898)
```
APE w.r.t. full transformation (unit-less)
(not aligned)

       max	128.180486
      mean	38.566743
    median	28.547089
       min	0.000002
      rmse	51.926611
       sse	12238836.862241
       std	34.770380
```
综上可以看出，在没有闭环检测的情况下，无论是NDT，还是ICP激光里程计都会发生漂移，而且Z方向漂移都很大（因为在平面运动，有退化现象？），从效果上来看NDT > GN-ICP > SVD-ICP。