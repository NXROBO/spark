#!/bin/bash

BASEPATH=$(cd `dirname $0`; pwd)
gnome-terminal -x bash -c "$BASEPATH/send_topic.sh"


