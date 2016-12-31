# Spark

This repository contains the ROS wrapper of Sparks's driver plus various ROS applications.This is a meta-package.

## Table of Contents

1. [Packages Overview](#packages-overview)
2. [Usage](#usage)
3. [Mirror](#mirror)

## Packages Overview

* ***spark_v03*** : spark 0.3 driver which include base driver, camera driver, robot description, teleop package, follow person package and so on.
* ***tools*** : it contains the 3th part openni2 driver which used in camera driver.
* ***doc*** : it shows that how to compile and use this meta-package.

## Usage

### Prequirement

* System:	Ubantu 14.04
* ROS Version:	Indigo(Desktop-Full Install) 

### Compile

```yaml
#make a workspace
mkdir sparkws/src
cd sparkws/src
git clone https://github.com/NXROBO/spark.git
cd ..
#installl dependence package
./src/spark/doc/install.sh
#Compile
catkin_make
#Install
catkin_make install
```
If everything goes fine, test the follow person example as follow:
```yaml
./install/follow_run.sh
```

# Mirror

We also provide a downloadable mirror where all environments have been configured.
*  Download address: [spark_mirror](http://pan.baidu.com/s/1i4ZlH4p)

