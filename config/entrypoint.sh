#!/bin/bash
set -e 
source $HOME/.bashrc 
cd /home/mycobot/catkin_ws
exec bash -i -c $@
