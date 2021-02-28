# Lecture5-Homework

按照不需要转台标定方法中所给出的内参模型及残差模型,推导加速度计对应残差对加速度内参的
雅可比,在开源代码中按新的内参模型 (开源代码中加速度计内参模型是上三角,本课程模型是下
三角)改成解析式求导,并使用课程给定的仿真数据做验证。
## 雅可比推导


作业中使用了开源代码中的模型进行推导


加速度计安装误差


![](https://cdn.nlark.com/yuque/__latex/53d0a168ffc8d0b86e89ebee94a0d7c2.svg#card=math&code=T%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%201%20%26%20-T_%7Byz%7D%20%26%20T_%7Bzy%7D%20%5C%5C%0A%20%20%20%20T_%7Bxz%7D%20%26%201%20%26%20-T_%7Bzx%7D%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7D%20%26%20T_%7Byx%7D%20%26%201%0A%5Cend%7Bbmatrix%7D%0A&height=63&width=183)


刻度系数误差


![](https://g.yuque.com/gr/latex?K%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20Kx%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%20Ky%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%200%20%26%20Kz%0A%5Cend%7Bbmatrix%7D%0A#card=math&code=K%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20Kx%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%20Ky%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%200%20%26%20Kz%0A%5Cend%7Bbmatrix%7D%0A)


偏置


![](https://g.yuque.com/gr/latex?bias%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20bx%20%26%20by%20%26%20bz%0A%5Cend%7Bbmatrix%7D%0A#card=math&code=bias%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20bx%20%26%20by%20%26%20bz%0A%5Cend%7Bbmatrix%7D%0A)


真实值与加速度计输出值关系为：


![](https://g.yuque.com/gr/latex?X%5E%7B%5Cprime%7D%20%3D%20T%20*%20K%20*%20(X%20-%20bias)%0A#card=math&code=X%5E%7B%5Cprime%7D%20%3D%20T%20%2A%20K%20%2A%20%28X%20-%20bias%29%0A)


其中![](https://g.yuque.com/gr/latex?X#card=math&code=X)为真实值，![](https://g.yuque.com/gr/latex?X%5E%7B%5Cprime%7D#card=math&code=X%5E%7B%5Cprime%7D) 为加速度计输出值


当地重力矢量为![](https://g.yuque.com/gr/latex?g%3D%5Cbegin%7Bbmatrix%7D%0A%20%20%20%200%20%26%200%20%26%20g_0%0A%5Cend%7Bbmatrix%7D#card=math&code=g%3D%5Cbegin%7Bbmatrix%7D%0A%20%20%20%200%20%26%200%20%26%20g_0%0A%5Cend%7Bbmatrix%7D)


则有残差函数


![](https://g.yuque.com/gr/latex?f(%5Ctheta%5E%7Bacc%7D)%20%3D%20%7C%7Cg%7C%7C_%7B2%7D%20-%20%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%0A#card=math&code=f%28%5Ctheta%5E%7Bacc%7D%29%20%3D%20%7C%7Cg%7C%7C_%7B2%7D%20-%20%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%0A)


残差对加速度计估计参数的偏导为


![](https://g.yuque.com/gr/latex?%5Cbegin%7Baligned%7D%0A%20%20%20%20%5Cfrac%7B%5Cpartial%7Bf%7D%7D%7B%5Ctheta%5E%7Bacc%7D%7D%20%3D%26%20%5Cfrac%7B%5Cpartial%7Bf%7D%7D%7B%5Cpartial%7B%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%7D%7D%5Cfrac%7B%5Cpartial%7B%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%7D%7D%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%5C%5C%0A%20%20%20%20%3D%20%26%20-%20%5Cfrac%7BX%5E%7B%5Cprime%7D%7D%7B%5Cbegin%7Bvmatrix%7DX%5E%7B%5Cprime%7D%5Cend%7Bvmatrix%7D%7D%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%0A%5Cend%7Baligned%7D%0A#card=math&code=%5Cbegin%7Baligned%7D%0A%20%20%20%20%5Cfrac%7B%5Cpartial%7Bf%7D%7D%7B%5Ctheta%5E%7Bacc%7D%7D%20%3D%26%20%5Cfrac%7B%5Cpartial%7Bf%7D%7D%7B%5Cpartial%7B%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%7D%7D%5Cfrac%7B%5Cpartial%7B%7C%7CX%5E%7B%5Cprime%7D%7C%7C_%7B2%7D%7D%7D%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%5C%5C%0A%20%20%20%20%3D%20%26%20-%20%5Cfrac%7BX%5E%7B%5Cprime%7D%7D%7B%5Cbegin%7Bvmatrix%7DX%5E%7B%5Cprime%7D%5Cend%7Bvmatrix%7D%7D%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%0A%5Cend%7Baligned%7D%0A)


安装误差取下三角模型


![](https://g.yuque.com/gr/latex?T%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%201%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20T_%7Bxz%7D%20%26%201%20%26%200%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7D%20%26%20T_%7Byx%7D%20%26%201%0A%5Cend%7Bbmatrix%7D%0A#card=math&code=T%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%201%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20T_%7Bxz%7D%20%26%201%20%26%200%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7D%20%26%20T_%7Byx%7D%20%26%201%0A%5Cend%7Bbmatrix%7D%0A)


则有


![](https://g.yuque.com/gr/latex?%5Cbegin%7Baligned%7D%0AX%5E%7B%5Cprime%7D%20%26%20%3D%20%20T%20*%20K%20*%20(X%20-%20bias)%20%5C%5C%0A%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%201%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20T_%7Bxz%7D%20%26%201%20%26%200%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7D%20%26%20T_%7Byx%7D%20%26%201%0A%5Cend%7Bbmatrix%7D%0A%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20Kx%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%20Ky%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%200%20%26%20Kz%0A%5Cend%7Bbmatrix%7D%0A%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20A_x%20-%20b_x%20%5C%5C%0A%20%20%20%20A_y%20-%20b_y%20%5C%5C%0A%20%20%20%20A_z%20-%20b_z%0A%5Cend%7Bbmatrix%7D%0A%0A%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20K_x(A_x-b_x)%20%5C%5C%0A%20%20%20%20T_%7Bxz%7DK_x(A_x-b_x)%2BK_y(A_y-b_y)%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7DK_x(A_x-b_x)%2BT_%7Byx%7DK_y(A_y-b_y)%2BK_z(A_z-b_z)%0A%5Cend%7Bbmatrix%7D%0A%0A%5Cend%7Baligned%7D%0A#card=math&code=%5Cbegin%7Baligned%7D%0AX%5E%7B%5Cprime%7D%20%26%20%3D%20%20T%20%2A%20K%20%2A%20%28X%20-%20bias%29%20%5C%5C%0A%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%201%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20T_%7Bxz%7D%20%26%201%20%26%200%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7D%20%26%20T_%7Byx%7D%20%26%201%0A%5Cend%7Bbmatrix%7D%0A%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20Kx%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%20Ky%20%26%200%20%5C%5C%0A%20%20%20%200%20%26%200%20%26%20Kz%0A%5Cend%7Bbmatrix%7D%0A%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20A_x%20-%20b_x%20%5C%5C%0A%20%20%20%20A_y%20-%20b_y%20%5C%5C%0A%20%20%20%20A_z%20-%20b_z%0A%5Cend%7Bbmatrix%7D%0A%0A%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20K_x%28A_x-b_x%29%20%5C%5C%0A%20%20%20%20T_%7Bxz%7DK_x%28A_x-b_x%29%2BK_y%28A_y-b_y%29%20%5C%5C%0A%20%20%20%20-T_%7Bxy%7DK_x%28A_x-b_x%29%2BT_%7Byx%7DK_y%28A_y-b_y%29%2BK_z%28A_z-b_z%29%0A%5Cend%7Bbmatrix%7D%0A%0A%5Cend%7Baligned%7D%0A)


![](https://g.yuque.com/gr/latex?%5Cbegin%7Baligned%7D%0A%20%20%20%20%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%20%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%20%0A%20%20%20%20%5Cend%7Bbmatrix%7D%20%5C%5C%0A%20%20%20%20%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%200%20%26%200%20%26%200%20%26%20A_x-b_x%20%26%200%20%26%200%20%26%20-K_x%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20%20%20%20%20K_x(A_x-b_x)%20%26%200%20%26%200%20%26%20T_%7Bxz%7D(A_x-b_x)%20%26%20A_y-b_y%20%26%200%20%26%20-T_%7Bxz%7DK_x%20%26%20-K_y%20%26%200%20%5C%5C%0A%20%20%20%20%20%20%20%200%20%26%20-K_x(A_x-b_x)%20%26%20K_y(A_y-b_y)%20%26%20-T_%7Bxy%7D(A_x-b_x)%20%26%20T_%7Byx%7D(A_y-b_y)%20%26%20A_z-b_z%20%26%20T_%7Bxy%7DK_x%20%26%20-T_%7Byx%7DK_y%20%26%20-K_%7Bz%7D%0A%20%20%20%20%5Cend%7Bbmatrix%7D%0A%5Cend%7Baligned%7D%0A#card=math&code=%5Cbegin%7Baligned%7D%0A%20%20%20%20%5Cfrac%7B%5Cpartial%7BX%5E%7B%5Cprime%7D%7D%7D%7B%5Cpartial%7B%5Ctheta%5E%7Bacc%7D%7D%7D%20%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Bxy%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BT_%7Byx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7BK_%7Bz%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bx%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7By%7D%7D%7D%20%26%20%0A%20%20%20%20%20%20%20%20%5Cfrac%7B%5Cpartial%7Be_x%7D%7D%7B%5Cpartial%7Bb_%7Bz%7D%7D%7D%20%5C%5C%20%0A%20%20%20%20%5Cend%7Bbmatrix%7D%20%5C%5C%0A%20%20%20%20%26%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%20%20%20%20%20%20%20%200%20%26%200%20%26%200%20%26%20A_x-b_x%20%26%200%20%26%200%20%26%20-K_x%20%26%200%20%26%200%20%5C%5C%0A%20%20%20%20%20%20%20%20K_x%28A_x-b_x%29%20%26%200%20%26%200%20%26%20T_%7Bxz%7D%28A_x-b_x%29%20%26%20A_y-b_y%20%26%200%20%26%20-T_%7Bxz%7DK_x%20%26%20-K_y%20%26%200%20%5C%5C%0A%20%20%20%20%20%20%20%200%20%26%20-K_x%28A_x-b_x%29%20%26%20K_y%28A_y-b_y%29%20%26%20-T_%7Bxy%7D%28A_x-b_x%29%20%26%20T_%7Byx%7D%28A_y-b_y%29%20%26%20A_z-b_z%20%26%20T_%7Bxy%7DK_x%20%26%20-T_%7Byx%7DK_y%20%26%20-K_%7Bz%7D%0A%20%20%20%20%5Cend%7Bbmatrix%7D%0A%5Cend%7Baligned%7D%0A)


## Ceres 实现


```cpp
// 解析求导
class AnalyticMultiPosAccResidual : public ceres::SizedCostFunction<1, 9> {
  public:
    AnalyticMultiPosAccResidual(const double &g_mag, const Eigen::Matrix<double, 3, 1> &sample)
      : g_mag_(g_mag), sample_(sample) {}
    
    virtual ~AnalyticMultiPosAccResidual() {}

    virtual bool Evaluate(double const* const* params, double *residuals, double **jacobians) const {
        const double Txz = params[0][0];
        const double Txy = params[0][1];
        const double Tyx = params[0][2];
        const double Kx = params[0][3];
        const double Ky = params[0][4];
        const double Kz = params[0][5];
        const double bx = params[0][6];
        const double by = params[0][7];
        const double bz = params[0][8];

        // lower triad model
        Eigen::Matrix<double, 3, 3> T;
        T << 1, 0, 0, 
             Txz, 1, 0, 
             -Txy, Tyx, 1;
        
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = Kx;
        K(1, 1) = Ky;
        K(2, 2) = Kz;

        Eigen::Vector3d bias(bx, by, bz);

        Eigen::Vector3d calib_samp = T * K * (sample_.col(0) - bias);

        residuals[0] = g_mag_ - calib_samp.norm();

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                Eigen::Vector3d x_xnorm = calib_samp / (calib_samp.norm());
                // 测量值减去bias
                Eigen::Vector3d v = sample_ - bias;
                // calib_samp 对参数的偏导
                Eigen::Matrix<double, 3, 9> J_theta = Eigen::Matrix<double, 3, 9>::Zero();
                J_theta(0, 3) = v(0);
                J_theta(0, 6) = -Kx;
                J_theta(1, 0) = Kx * v(0);
                J_theta(1, 3) = Txz * v(0);
                J_theta(1, 4) = v(1);
                J_theta(1, 6) = -Txz * Kx;
                J_theta(1, 7) = -Ky;
                J_theta(2, 1) = -Kx * v(0);
                J_theta(2, 2) = Ky * v(1);
                J_theta(2, 3) = -Txy * v(0);
                J_theta(2, 4) = Tyx * v(1);
                J_theta(2, 5) = v(2);
                J_theta(2, 6) = Txy * Kx;
                J_theta(2, 7) = -Tyx * Ky;
                J_theta(2, 8) = -Kz;
                Eigen::Matrix<double, 1, 9> Jaco = - x_xnorm.transpose() * J_theta;
                jacobians[0][0] = Jaco(0, 0);
                jacobians[0][1] = Jaco(0, 1);
                jacobians[0][2] = Jaco(0, 2);
                jacobians[0][3] = Jaco(0, 3);
                jacobians[0][4] = Jaco(0, 4);
                jacobians[0][5] = Jaco(0, 5);
                jacobians[0][6] = Jaco(0, 6);
                jacobians[0][7] = Jaco(0, 7);
                jacobians[0][8] = Jaco(0, 8);
            }
        }
        return true;
    }

  private:
    const double g_mag_;
    const Eigen::Matrix< double, 3 , 1> sample_;
};

// 添加参数
ceres::CostFunction* cost_function = new AnalyticMultiPosAccResidual(g_mag_, static_samples[i].data());
problem.AddResidualBlock(cost_function, NULL, acc_calib_params.data());
```


## 结果对比


- 开源代码自动求导标定结果
```bash
Accelerometers calibration: Better calibration obtained using threshold multiplier 6 with residual 0.120131
Misalignment Matrix
          1  -0.0033593 -0.00890639
          0           1  -0.0213341
        -0           0           1
Scale Matrix
0.00241278          0          0
        0 0.00242712          0
        0          0 0.00241168
Bias Vector
33124.2
33275.2
32364.4

Accelerometers calibration: inverse scale factors:
414.459
412.01
414.649

Gyroscopes calibration: residual 0.00150696
Misalignment Matrix
        1 0.00593634 0.00111101
0.00808812          1 -0.0535569
0.0253067 -0.0025513          1
Scale Matrix
0.000209295           0           0
          0 0.000209899           0
          0           0 0.000209483
Bias Vector
32777.1
32459.8
32511.8

Gyroscopes calibration: inverse scale factors:
4777.96
4764.21
4773.66
```


- 解析求导标定结果
```bash
Accelerometers calibration: Better calibration obtained using threshold multiplier 6 with residual 0.120131
Misalignment Matrix
          1          -0           0
-0.00354989           1          -0
-0.00890444  -0.0213032           1
Scale Matrix
0.00241267          0          0
         0 0.00242659          0
         0          0 0.00241232
Bias Vector
33124.2
33275.2
32364.4

Accelerometers calibration: inverse scale factors:
414.478
412.102
414.538

Gyroscopes calibration: residual 0.00150696
Misalignment Matrix
        1 0.00927517 0.00990014
0.00507442          1 -0.0322229
0.0162201 -0.0239393          1
Scale Matrix
0.000209338           0           0
        0 0.000209834           0
        0           0 0.000209664
Bias Vector
32777.1
32459.8
32511.8

Gyroscopes calibration: inverse scale factors:
4776.96
4765.68
4769.53
```








