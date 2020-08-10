#!/bin/bash

Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 是否现在开始视觉抓取？"
echo -e "${Info} 开始请按按任意键，否则退出请输入：Ctrl + c    " 
echo && stty erase '^H' && read -p "按任意键开始：" 
gnome-terminal -x bash -c "rosservice call /s_carry_object 'type: 1'"
