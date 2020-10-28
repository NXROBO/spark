#!/bin/bash

Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 是否已经把绿色框对准了机械臂的红色标定点？"
echo -e "${Info} 确定请按按任意键，否则退出请输入：Ctrl + c    " 
echo && stty erase '^H' && read -p "按任意键确定：" 

rostopic pub /start_topic std_msgs/String "data: 'start'"
