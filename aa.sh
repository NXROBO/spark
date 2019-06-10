PROJECTPATH=$(cd `dirname $0`; pwd)
source ${PROJECTPATH}/devel/setup.bash
roslaunch dobot spark_carry_cal_cv3.launch
