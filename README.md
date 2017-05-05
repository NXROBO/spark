# Spark

This repository contains the ROS wrapper of Sparks's driver plus various ROS applications.This is a meta-package.

## Table of Contents

1. [Packages Overview](#packages-overview)
2. [Usage](#usage)
3. [Mirror](#mirror)

## Packages Overview

* ***spark_v03*** : spark 0.3 driver including base driver, camera driver, robot description, teleop package, and follow person package and so on.
* ***tools*** : it contains the 3rd part openni2 driver which camera driver uses.
* ***doc*** : it shows that how to compile and use this meta-package.

## Usage

### Prequirement

* System:	Ubantu 14.04
* ROS Version:	Indigo(Desktop-Full Install) 

### Compile

Build this compile with the following steps:
```yaml
#make a workspace
mkdir -p sparkws/src
cd sparkws/src
git clone https://github.com/NXROBO/spark.git
git checkout spark-03  //if your spark is the 0.3 base, please type the command:git checkout spark-04
cd ..
#installl dependence package
./src/spark/doc/install.sh
#Compile
catkin_make
#Install
catkin_make install
```
If everything goes fine, test the follow-person example as follow:
```yaml
./install/follow_run.sh
```

# Mirror

We also provide a downloadable mirror whose all environments have been configured.
*  Download address: [spark_mirror](http://pan.baidu.com/s/1i4ZlH4p)

