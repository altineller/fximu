#!/bin/bash
USER_NAME=`whoami`;
sudo chown $USER_NAME:$USER_NAME /dev/ttyCAL0;
sudo chown $USER_NAME:$USER_NAME /dev/ttyCAL1;
