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