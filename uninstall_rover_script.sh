#!/bin/bash
WORKSPACE_NAME=test_workspace
BOLDRED="\e[1;31m"
ENDCOLOR="\e[0m"
RED="\e[31m"
GREEN="\e[32m"

print_boldred() {
    echo -e "$BOLDRED${1} $ENDCOLOR"
}
print_red() {
    echo -e "$RED${1} $ENDCOLOR"
}
print_green() {
    echo -e "$GREEN${1} $ENDCOLOR"
}

# Prompt the user to confirm uninstall
while true; do
    print_boldred "CAUTION! This script will uninstall all services from Rover Robotics."
    print_boldred "It will not remove the rover_workspace packages. You may do so on your own."
    printf "Are you sure you would like to uninstall? [y/n]: "
    read confirm_uninstall
    case $confirm_uninstall in
        [Yy]* ) confirm_uninstall=true; break;;
        [Nn]* ) confirm_uninstall=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo ""
if [ "$confirm_uninstall" = true ]; then
    if [ -f /usr/sbin/roverrobotics ]; then
        sudo rm /usr/sbin/roverrobotics
        if [ $? -ne 0 ]; then
            print_red "Unable to remove /usr/sbin/roverrobotics"
        else
            print_green "Successfully removed /usr/sbin/roverrobotics"
        fi
    fi

    if [ -f /etc/systemd/system/roverrobotics.service ]; then
        sudo systemctl stop roverrobotics.service 
        sudo systemctl disable roverrobotics.service
        sudo rm /etc/systemd/system/roverrobotics.service
        sudo systemctl daemon-reload
        if [ $? -ne 0 ]; then
            print_red "Unable to remove /etc/systemd/system/roverrobotics.service"
        else
            print_green "Successfully removed /etc/systemd/system/roverrobotics.service"
        fi
    fi

    if [ -f /usr/sbin/enablecan ]; then
        sudo rm /usr/sbin/enablecan
        if [ $? -ne 0 ]; then
            print_red "Unable to remove /usr/sbin/enablecan"
        else
            print_green "Successfully removed /usr/sbin/enablecan"
        fi
    fi

    if [ -f /etc/systemd/system/can.service ]; then
        sudo systemctl stop can.service 
        sudo systemctl disable can.service
        sudo rm /etc/systemd/system/can.service
        sudo systemctl daemon-reload
        if [ $? -ne 0 ]; then
            print_red "Unable to remove /etc/systemd/system/can.service"
        else
            print_green "Successfully removed /etc/systemd/system/can.service"
        fi
    fi

    if [ -f /etc/udev/rules.d/55-roverrobotics.rules ]; then
        sudo rm /etc/udev/rules.d/55-roverrobotics.rules
        if [ $? -ne 0 ]; then
            print_red "Unable to remove /etc/udev/rules.d/55-roverrobotics.rules"
        else
            print_green "Successfully removed /etc/udev/rules.d/55-roverrobotics.rules"
        fi
    fi
    grep -F "source ~/$WORKSPACE_NAME/install/setup.bash" ~/.bashrc &&
    sed -i "\|source ~/$WORKSPACE_NAME/install/setup.bash|d" ~/.bashrc > /dev/null
fi

