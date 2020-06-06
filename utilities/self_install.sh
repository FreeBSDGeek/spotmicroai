#!/bin/bash

sudo apt update -y

sudo apt dist-upgrade -y

sudo apt autoremove -y

sudo apt install git python3-venv sshpass i2c-tools python-smbus joystick xboxdrv -y

grep -qxF 'options bluetooth disable_ertm=Y' /etc/modprobe.d/bluetooth.conf || echo 'options bluetooth disable_ertm=Y' | sudo tee -a /etc/modprobe.d/bluetooth.conf
cat /etc/modprobe.d/bluetooth.conf

cd ~ || exit
git clone https://github.com/FreeBSDGeek/spotmicro.git spotmicroai
git checkout development
cd spotmicroai || exit

find . -type f -iname "*.sh" -exec chmod +x {} \;

~/spotmicroai/utilities/activate.sh
