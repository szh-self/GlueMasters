#!/bin/bash

# 设置工作空间
source install/setup.bash

# 运行launch文件并保持终端
ros2 launch gluemaster gluemaster.launch.py
echo ""
echo "所有节点已停止运行"
echo "按回车键关闭此窗口..."
read -r