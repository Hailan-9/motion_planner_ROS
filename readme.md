



## 使用gdb进行调试
ros中使用gdb调试，主要分为两种，一种是通过rosrun，另一种是通过roslaunch。
使用roslaunch时，需要在启动的节点语句最后，加上launch-prefix="xterm -e gdb --args"