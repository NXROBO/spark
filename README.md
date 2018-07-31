# Spark

This repository contains the ROS wrapper of Sparks's driver plus various ROS applications.This is a meta-package.

## Table of Contents

1. [Update Log](#update-log)
2. [Packages Overview](#packages-overview)
3. [Usage](#usage)
4. [Mirror](#mirror)

## Update Log

* Raise the stack so that the laidar can be added on it.
* Update the pre install packages so that the navigation and gmapping can be run.

## Packages Overview

* ***spark_v05*** : spark 0.5 driver including base driver, camera driver, robot description, teleop package, and follow person package and so on.
* ***tools*** : it contains the 3rd part openni2 driver which camera driver uses.
* ***doc*** : it shows that how to compile and use this meta-package.

## Usage

### Prequirement

* System:	Ubantu 14.04
* ROS Version:	Indigo(Desktop-Full Install) 

### Compile

Build this compile with the following steps:
```yaml
git clone https://github.com/NXROBO/spark.git

#install dependence package
cd spark
./doc/install.sh

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

