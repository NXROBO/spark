PROJECTPATH=$(cd `dirname $0`; pwd)
source ${PROJECTPATH}/devel/setup.bash
roslaunch dobot spark_carry_object_only_cv3.launch 
