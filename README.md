# Spark

This repository contains the ROS wrapper of Sparks's driver plus various ROS applications.This is a meta-package.

## Table of Contents

1. [Packages Overview](#packages-overview)
2. [Usage](#usage)

## Packages Overview

* ***spark_v03*** : Spark驱动
* ***tools*** : it contains the 3th part camera driver package.
* ***doc*** : it shows that how to compile the meta-package.


## Usage

### Prequirement
System:		Ubantu 14.04
ROS Version:	Indigo(Desktop-Full Install) 

### Compile

```bash
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
```bash
./install/follow_run.sh
```

