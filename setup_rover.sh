#!/bin/bash
ROS_DISTRO=humble
ROVER_REPO=https://github.com/RoverRobotics/ros2_roverrobotics_development.git
WORKSPACE_NAME=test_workspace
CURRENT_DIR=${PWD}
BASEDIR=$CURRENT_DIR

RED="\e[31m"
GREEN="\e[32m"
BOLD="\e[1m"
ITALIC="\e[3m"
ITALICBLUE="\e[3;94m"
BOLDBLUE="\e[1;94m"
ENDCOLOR="\e[0m"

# Helper functions

print_red() {
    echo -e "$RED${1} $ENDCOLOR"
}
print_green() {
    echo -e "$GREEN${1} $ENDCOLOR"
}
print_bold() {
    echo -e "$BOLD${1} $ENDCOLOR"
}
print_italic() {
    echo -e "$ITALICBLUE${1} $ENDCOLOR"
}
print_boldblue(){
    echo -e "$BOLDBLUE${1} $ENDCOLOR"
}
print_next_install() {
    install_number=$((install_number+1))
    print_bold "[$install_number/$install_total]: ${1}"
}

print_install_settings() {
    print_bold "====================================="
    print_boldblue "                                     "
    print_bold "Installation settings:               "
    print_bold "----------------------------         "
    print_boldblue "Robot type:   $device_type           "
    print_boldblue "Repo:         $install_repo          "
    print_boldblue "Service:      $install_service       "
    print_boldblue "Udev:         $install_udev          "
    print_boldblue "                                     "
    print_bold "====================================="
    echo ""
}


# Define the install service functions
create_startup_script() {
    local robot_type=$1
    cat << EOF2 | sudo tee /usr/sbin/roverrobotics
#!/bin/bash
source ~/rover_workspace/install/setup.sh
ros2 launch roverrobotics_driver ${robot_type}_teleop.launch.py
PID=\$!
wait "\$PID"
EOF2

    sudo chmod +x /usr/sbin/roverrobotics
}

create_startup_service() {
    cat << EOF3 | sudo tee /etc/systemd/system/roverrobotics.service
    [Service]
    Type=simple
    User=$USER
    ExecStart=/bin/bash /usr/sbin/roverrobotics
    [Install]
    WantedBy=multi-user.target
EOF3

    sudo systemctl enable roverrobotics.service
}

create_can_service() {      
    cat << EOF4 | sudo tee /usr/sbin/enablecan
#!/bin/bash
sudo ip link set can0 type can bitrate 500000 sjw 127 dbitrate 2000000 dsjw 15 berr-reporting on fd on
sudo ip link set up can0
EOF4

    sudo chmod +x /usr/sbin/enablecan

    cat << EOF5 | sudo tee /etc/systemd/system/can.service
[Service]
Type=simple
User=root
ExecStart=/usr/sbin/enablecan
[Install]
WantedBy=multi-user.target
EOF5

    sudo systemctl enable can.service

    sudo ip link set can0 type can bitrate 500000 sjw 127 dbitrate 2000000 dsjw 15 berr-reporting on fd on
    sudo ip link set up can0
}


# Define the install_ros_packages function
install_ros_packages() {
    local error_count=0
    sudo apt-get install -y ros-$ROS_DISTRO-slam-toolbox > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-slam-toolbox."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-slam-toolbox: Success"
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-navigation2 > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-navigation2."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-navigation2: Success"
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-nav2-bringup > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-nav2-bringup."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-nav2-bringup: Success"
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-joint-state-publisher > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-joint-state-publisher."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-joint-state-publisher: Success"
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-xacro > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-xacro."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-xacro: Success"
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-joy-linux > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing ros-$ROS_DISTRO-joy-linux."
        error_count=$((error_count+1))
    else
        print_green "ros-$ROS_DISTRO-joy-linux: Success"
    fi
    sudo apt-get install -y git > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing git."
        error_count=$((error_count+1))
    else
        print_green "git: Success"
    fi
    sudo apt-get install -y net-tools > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Error encountered while installing net-tools."
        error_count=$((error_count+1))
    else
        print_green "net-tools: Success"
    fi
    if [ $error_count -gt 0 ]; then
        echo ""
        print_red "Finished checking/installing packages with $error_count error(s)."
        print_red "Check that ROS2 is installed correctly and repository sources are correct."
        return 1
    else
        echo ""
        print_green "Finished checking/installing packages successfully."
        return 0
    fi
}

clear

# Prompt the user for device type
while true; do
    printf "Enter the robot type [(1) indoor_miti, (2) miti, (3) mini, (4) zero, (5) pro]: "
    read device_type

    case "$device_type" in
        "1")
            device_type="indoor_miti"
            ;;
        "2")
            device_type="miti"
            ;;
        "3")
            device_type="mini"
            ;;
        "4")
            device_type="zero"
            ;;
        "5")
            device_type="pro"
            ;;
    esac


    # Check if the entered device type is valid
    if [ "$device_type" != "indoor_miti" ] && [ "$device_type" != "miti" ] && [ "$device_type" != "mini" ] && [ "$device_type" != "zero" ] && [ "$device_type" != "pro" ]; then
        print_red "Invalid robot type."
    else
        break
    fi
done

# Prompt the user to decide about installing ros2 drivers
while true; do
    printf "Would you like to install the Rover Robotics ros2 repository? [y/n]: "
    read install_repo
    case $install_repo in
        [Yy]* ) install_repo=true; break;;
        [Nn]* ) install_repo=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# Prompt the user to decide about installing automatic start service
while true; do
    printf "Would you like to install the automatic start service? [y/n]: "
    read install_service
    case $install_service in
        [Yy]* ) install_service=true; break;;
        [Nn]* ) install_service=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done


# Prompt the user to decide about installing the udev rules
while true; do
    printf "Would you like to install the udev rules? [y/n]: "
    read install_udev
    case $install_udev in
        [Yy]* ) install_udev=true; break;;
        [Nn]* ) install_udev=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

install_number=0
install_total=2
install_can=false

if [ "$install_repo" = true ]; then
    install_total=$((install_total+1))
fi

if [ "$install_service" = true ]; then
    install_total=$((install_total+1))
fi

if [ "$install_udev" = true ]; then
    install_total=$((install_total+1))
fi

if [ "$device_type" = "indoor_miti" ] || [ "$device_type" = "miti" ] || [ "$device_type" = "mini" ]; then
    if [ ! -f /etc/systemd/system/can.service ]; then
        install_can=true
    fi
fi

# Prompt the user to decide about installing CAN driver
if [ "$install_can" = true ]; then
    while true; do
        printf "Is the Rover $device_type connected via CAN-TO-USB? [y/n]: "
        read rover_can
        case $rover_can in
            [Yy]* ) install_can=true; install_total=$((install_total+1)); break;;
            [Nn]* ) install_can=false; break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

clear

print_install_settings

# ROS Packages
print_next_install "Checking/Installing dependent packages"
install_ros_packages
echo ""

if [ "$install_repo" = true ]; then
    print_next_install "Installing the Rover Robotics ROS2 packages"
    
    print_italic "Setting up rover workspace in /home/${WORKSPACE_NAME}"
    mkdir -p ~/$WORKSPACE_NAME/src > /dev/null
    cd ~/$WORKSPACE_NAME/src > /dev/null

    echo ""
    print_italic "Cloning Rover Robotics ROS2 packages into /home/${WORKSPACE_NAME}/src"
    echo ""
    git clone $ROVER_REPO -b $ROS_DISTRO > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Failed to clone Rover Robotics ROS2 packages"
    else
        print_green "Successfully cloned packages."
    fi

    echo ""
    print_italic "Building Rover Robotics ROS2 packages"
    cd ~/$WORKSPACE_NAME > /dev/null
    source /opt/ros/$ROS_DISTRO/setup.sh > /dev/null
    colcon build
    if [ $? -ne 0 ]; then
        print_red "Failed to build Rover Robotics ROS2 packages"
    else
        print_green "Successfully built packages."
        grep -F "source ~/$WORKSPACE_NAME/install/setup.bash" ~/.bashrc ||
        echo "source ~/$WORKSPACE_NAME/install/setup.bash" >> ~/.bashrc

        source ~/$WORKSPACE_NAME/install/setup.bash > /dev/null
    fi
    echo ""
fi

if [ "$install_can" = true ]; then
    print_next_install "Installing the CAN services"
    print_italic "Setting up CAN for Rover $device_type"
    create_can_service > /dev/null

    if [ ! -f /etc/systemd/system/can.service ]; then
        print_red "Failed to create can.service @ /etc/systemd/system/can.service"
    else
        print_green "Successfully created can.service"
    fi

    if [ ! -f /usr/sbin/enablecan ]; then
        print_red "Failed to create enablecan @ /usr/sbin/enablecan"
    else
        print_green "Successfully created enablecan.sh"
    fi

    if ifconfig | grep -q can0; then
        print_green "Set up can0 successfully"
    else
        print_red "Failed to setup can0 device. Check computer is connected to robot and robot is powered on."
    fi
    echo ""
fi

if [ "$install_service" = true ]; then
    print_next_install "Installing the automatic start service"

    print_italic "Creating startup script..."
    create_startup_script $device_type > /dev/null
    if [ -f /usr/sbin/roverrobotics ]; then
        print_green "Succeeded in creating the startup script."
    else
        print_red "Failed creating the startup script. File: /usr/sbin/roverrobotics does not exist."
    fi

    echo ""
    print_italic "Creating startup service..."
    create_startup_service > /dev/null
    if [ -f /etc/systemd/system/roverrobotics.service ]; then
        print_green "Succeeded in creating the startup service."
    else
        print_red "Failed creating the startup service. File: /etc/systemd/system/roverrobotics.service does not exist."
    fi

    echo ""
fi

if [ "$install_udev" = true ]; then
    print_next_install "Installing the udev rules"
    
    print_italic "Copying Udev rules into /etc/udev/rules.d/55-roverrobotics.rules"
    sudo cp $BASEDIR/udev/55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules > /dev/null
    if [ $? -ne 0 ]; then
        print_red "Failed to copy Udev rules into /etc/udev/rules.d/55-roverrobotics.rules"
    else
        print_green "Successfully copied udev rules"

        echo ""
        print_italic "Reloading udev rules"
        sudo udevadm control --reload-rules > /dev/null
        if [ $? -ne 0 ]; then
            print_red "Failed to reload udev rules"
        else
            print_green "Successfully reloaded rules"
        fi

        echo ""
        print_italic "Triggering udev rules"
        sudo udevadm trigger > /dev/null
        if [ $? -ne 0 ]; then
            print_red "Failed to trigger udevadm"
        else
            print_green "Triggered udev rules. This works most of the time but you may need to restart."
        fi
    fi

    echo ""
fi


# Restarts the services if they exist
if [ -f /etc/systemd/system/roverrobotics.service ] || [ -f /etc/systemd/system/can.service ]; then
    print_next_install "Restarting services for convenience"
    echo ""
    if [ -f /etc/systemd/system/can.service ]; then
        sudo systemctl restart can.service
        if [ $? -ne 0 ]; then
            print_red "Failed to restart can.service"
        else
            print_green "Restarted can.service"
        fi
    fi
    if [ -f /etc/systemd/system/roverrobotics.service ]; then
        sudo systemctl restart roverrobotics.service
        if [ $? -ne 0 ]; then
            print_red "Failed to restart roverrobotics.service"
        else
            print_green "Restarted roverrobotics.service"
        fi
    fi
    echo ""
fi  
