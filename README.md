# ReconROS
Easy to use framework for ROS2 FPGA-based hardware acceleration

based on [ReconOS](http://reconos.de) 
## Prepare the FPGA board

You have to install ROS2 and ReconOS on your FPGA board. For getting started faster, there are preinstalled images:

[Pynq-Board](https://drive.google.com/open?id=1jpM5JdEsSBhS2G0khN9klwKmpMt_bnnn)

The easiest way to burn the image on the sd card would be to use [balena etcher](https://www.balena.io/etcher/). You can also use linux dd tool eg e.g. 

```
dd if=/image.img of=/dev/<<yoursdcard>> bs=4M
```

you will also need an ethernet cable, and would need to configure your terminal to have a local ip in the range of 192.168.2.xx(Netmask 255.255.255.0) except 99; This is the ip of pynq board. 

Configure the board to boot using SD card, using the jumper J4.

configure a serial terminal program. The COM port usually will be configured as 

/dev/ttyUSB1 -serial -sercfg 115200,8,n,1,N

If everything works fine, you will see the device boot, and it will then automatically log you in as 
xilinx.

## Installing
ReconROS uses an emulated docker container for the compilation of the application and building costum ROS message packages. 

The first step for building ReconROS application is sourcing the environment by using the following command:

```
sudo apt install python3-colcon-common-extensions
```
This step is not only necessary for the installation process but also before compiling an application.

At the first time using ReconROS, the following command installs needed packages and builds the docker container for cross compiling.
```
$ bash tools/install.sh
```

Additionaly you might have to source

```
$ source /opt/ros/dashing/setup.bash

$ source /opt/Xilinx/Vivado/2017.1/settings64.sh
```

also, there might be some packages missing, you might have to enter the following command

```
$ sudo apt-get install g++-multilib
```

## Build and run the ROS demo
There are three sort demos available. The demo used in the following are based on pub/sub communication. The other two demos use ROS2 actions and services for the client server communication. Both demos require a preceding command for building the costum ROS2 message package. 

```
$ rdk export_msg && rdk build_msg
```

### ROS sort demo
ROS sort demo is based on pub/sub communication, and follows the same procedure as ROS sort demo services to build and run server and client applications. After client application is build, it will continually send unsorted data, and will receive sorted data back.

### ROS sort demo services
The other two demos use ROS2 actions and services for the client server communication. Both demos require a preceding command for building the custom ROS2 message package. 

Navigate to /demos/ros\_sort\_demo\_services/server\_app/ and enter following commands

```
$ rdk export_msg && rdk build_msg
```

#### First Step: Server Application 

##### Building Server Application
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

##### copying files to pynq board

navigate to server_app/build.msg, and copy custom message package to the fpga using scp

```
$ scp message_package.zip xilinx@192.168.2.99:/
```
it will ask for password, which is the same as username: xilinx

navigate to server\_app/build.sw

before copying the application to the fpga, we have to make it executable and then copy it 

```
$ sudo chmod +x sortdemo

$ scp sortdemo xilinx@192.168.2.99:/
```
navigate to server\_app/build.hw/myReconOS.runs/impl\_1

```
scp design_1_wrapper.bit xilinx@192.168.2.99:/
```

##### running server application

on the serial terminal

```
$ sudo su
```

then do cd/ and enter ls to check if if message\_package.zip, design\_1\_wrapper.bit, and sortdemo have been copied on to the fpga

reconfigure the fpga using

```
$ cat design_1_wrapper.bit > /dev/xdevcfg
```
navigate to /opt/reconos and enter 

```
./reconos_init.sh
```
then 
```
$ source /opt/ros/dashing/setup.sh
```
navigate back, and create a directory mnt/project/build.msg/, and unzip message\_package.zip
```
$ unzip message_package.zip -d /mnt/project/build.msg/
```

then in mnt/project/build.msg/ enter command
```
$ source install/local_setup.sh
```

then run the server application by
```
./sortdemo 0 1
```
where 0 1 are the hardware parameters.

if everything works fine, you should see server message that it is waiting for new data

#### Second Step: Client Application
The x86 client application can be compiled by using the colcon build command.  Please note that ROS2 has to be installed on your machine.

first enter command

```
$ source /opt/ros/dashing/setup.sh
```

navigate to ros\_sort\_demo\_service/client\_app\_x86/sorter\_msgs and enter command

```
$ colcon build

$ source install/local_setup.bash
```

then do the same in ros\_sort\_demo\_service/client\_app\_x86/sorter\_client

and then finally

```
$ ros2 run sorter_client client
```
the client application on your machine which sends unsorted numbers to the server application and gets sorted data back.

For the ARM-based client application, the same procedure described for the server application have to be used. 

### ROS sort demo action

this demo follows the same steps as services except for running of Client application.

first enter command

```
$ source /opt/ros/dashing/setup.sh
```

navigate to client\_app\_x86/sorter\_msgs and enter command

```
$ colcon build

$ source install/local_setup.bash
```
then navigate back and enter command

```
$ python3 sort_action_client.py
```
you will see the progress messages both on server and client side.

## Publications

- [1] - [Christian Lienen, Marco Platzner, Bernhard Rinner "ReconROS: Flexible Hardware Acceleration for ROS2 Applications", International Conference on Field Programmable Technology (ICFPT), 2020.](https://ieeexplore.ieee.org/document/9415549)

- [2] - [Christian Lienen, Marco Platzner "Design of Distributed Reconfigurable Robotics Systems with ReconROS", ACM TRETS, 2021.](https://arxiv.org/abs/2107.07208)
