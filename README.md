# BaizeArm
robot arm

这是一个机械臂的项目，一个分布式控制系统的机械臂。

照片如下，长这个样子：

![机械臂照片1](https://github.com/Allen953/BaizeArm/blob/main/7.Photos%20%26%20Videos/joint.png)


主要用了两种驱动控制一体板：

用了一块BaizeCarBoard板子做第1个关节和末端夹爪的控制。
第一个关节采用步进电机做执行器，而末端夹爪采用舵机做执行器。

![关节控制驱动器1](https://github.com/Allen953/BaizeArm/blob/main/7.Photos%20%26%20Videos/IMG_20221201_094552.jpg)

这个驱动器可以驱动两路步进电机，同时又能驱动8路的舵机，因此用这块板子可以控制这个机械臂的两个自由度。包括一个轴和一个夹爪。

然后又用了两块Baize_Foc_v0.1板子来控制机械臂的第二、第三这两个自由度。这两个自由度都是由无刷电机做执行器的，于是刚好用这种板子来做分布式控制。

具体如下图：

![关节控制驱动器2](https://github.com/Allen953/BaizeArm/blob/main/7.Photos%20%26%20Videos/IMG_20221201_094700.jpg)


