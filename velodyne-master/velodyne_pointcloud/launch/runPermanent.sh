#!/bin/bash

pgrep rosmaster && echo roscore already runs
if test $? -eq 1; then roscore & fi
#pgrep startntripclient* && echo ntripclient already runs
#if test $? -eq 1; then $HOME/Hung/icps_ws/src/icps/scripts/startntripclient.sh & fi


