#!/bin/bash
sudo socat -d -d pty,raw,echo=0,link=/dev/ttyCAL0 pty,raw,echo=0,link=/dev/ttyCAL1;
