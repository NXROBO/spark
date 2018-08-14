#!/bin/bash
#!SPARK技术讨论与反馈群：8346256
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 是否开始保存当前的地图？"
echo -e "${Info} 开始请按按任意键，否则退出请输入：Ctrl + c    " 
echo && stty erase '^H' && read -p "按任意键开始：" 
CURRENTPATH=$(cd `dirname $0`; pwd)
gnome-terminal -x bash -c "rosrun map_server map_saver -f $CURRENTPATH/test_map"


