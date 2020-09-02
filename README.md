如何上传代码:
[https://jingyan.baidu.com/article/c1465413e4fc8b0bfdfc4c6f.html](http://)
# kuka_ws

## TIPS

 - 文件名不可带中文 , 否则不能同步到 GitLab

 - 使用 Roboware 打开工作空间的方法

    clone 后手动删除 src/Cmakelists.txt

    在 kuka_ws 目录下
    ```
    catkin_init_workspace src
    ```
 - gedit ~/.bashrc
    ```
    export ROS_IP=192.168.168.3
    export ROS_MASTER_URI=http://$ROS_IP:11311
    ```
 - `/home/lance/Workspaces/hxz_ws/src/darknet_ros/darknet_ros/CMakelists.txt` 中注释了 darknet 自己检查并下载官方权重文件的代码
