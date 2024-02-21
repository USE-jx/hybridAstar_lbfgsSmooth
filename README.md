# hybridAstar_lbfgsSmooth
hybrid astar with smooth, optimization solver is lbfgs

## 运行效果

![image-20231222222329069](./image-20231222222329069.png)

如图所示，黑色为障碍物，细蓝色带点的是混合A星搜索出来的路径点，红色是优化后的路径。

## 安装依赖

1 安装grid_map

```
sudo apt install ros-$ROS_DISTRO-grid-map
```

2 安装OMPL运动规划库

```
sydo apt install ros-$ROS_DISTRO-ompl
```

## 编译运行

创建一个工作空间

把src放入工作空间后`catkin_make`编译即可.
## 运行视频
https://www.bilibili.com/video/BV1CK411876z/?spm_id_from=333.999.0.0&vd_source=62bfb7720b0b2f9941f7f34210ba6a18
