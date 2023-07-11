# rover_install_scripts_ros2
ROS2 Install Scripts for Rover Robots

Clone this repo and then follow the instructions in the setup script.

```

git clone https://github.com/RoverRobotics/rover_install_scripts_ros2

cd rover_install_scripts_ros2

sudo chmod 777 setup_rover.sh

./setup_rover.sh

```


This install script will ask you which robot you wish to install and additionally asks if you want to create a roverrobotics.service, setup udev rules, etc. The service automatically starts on computer boot up and runs our robot driver. If you do not wish for it to automatically start, please decline the service creation. For the mini or miti, most have a can-to-usb converter that the script will set up the drivers for. If you wish, you can also plug a micro usb into the vesc port that controls the rear right hub motor. You must also change the config file for the mini or miti to use ``comm_type: serial`` and set the corresponding ``/dev/tty*`` port.

  

Once the install is finished you are good to go!
