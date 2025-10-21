#!/bin/bash
sleep 5
sudo raspi-config nonint do_expand_rootfs
sleep 3
sudo systemctl disable expand_rootfs.service
sudo reboot
