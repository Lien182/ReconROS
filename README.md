# ReconROS
Easy to use framework for ROS2 FPGA-based hardware acceleration

based on [ReconOS](http://reconos.de) 
## Prepare the FPGA board

You have to install ROS2 and ReconOS on your FPGA board. For getting started faster, there are preinstalled images:

[Pynq-Board](https://drive.google.com/open?id=1jpM5JdEsSBhS2G0khN9klwKmpMt_bnnn)


## Installing

First, set the environment variables
Install the environment and build the docker container for cross compiling
```
$ source tools/settings.sh
```


After that install the needed packages and build the docker container for cross compiling.
```
$ bash tools/install.sh
```
Additionally, you have to install Vivado (tested with 2017.1) and ROS2 (tested with Dashing)

## Build and run the ROS demo

### First Step: Server Application 
Navigate to the server demo folder. It contains the build.cfg file which is used by ReconROS. Before you can compile the application, you have to export the project:

```
$ rdk export_sw
```
and compile:

```
$ rdk build_sw
```
The hardware part is built analogously:
```
$ rdk export_hw
```
and synthesize:

```
$ rdk build_hw
```

Copy the output files (software binary and bitstream) to your FPGA board and run the server application there. 

### Second Step: Client Application
The client application is just compiled by use make in the client demo folder. Please note that ROS2 has to be installed on your machine.

After that you can run the client application on your machine which sends unsorted numbers to the server application and gets sorted data back.
