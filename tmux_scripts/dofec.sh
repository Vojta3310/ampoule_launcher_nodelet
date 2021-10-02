#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# change this to your liking
PROJECT_NAME=just_flying

# do not change this
MAIN_DIR=~/"bag_files"

# following commands will be executed first in each window
pre_input="mkdir -p $MAIN_DIR/$PROJECT_NAME"

# define commands
# 'name' 'command'
# DO NOT PUT SPACES IN THE NAMES
input=(
  'Rosbag' 'waitForOffboard; rosrun mrs_uav_general record.sh
'
  'Nimbro' 'waitForRos; roslaunch mrs_uav_general nimbro.launch
'
  'Sensors' 'waitForRos; roslaunch mrs_uav_general sensors.launch
'
  'RTK' 'waitForRos; roslaunch mrs_serial rtk.launch
'
  'Status' 'waitForRos; roslaunch mrs_uav_status status.launch
'
  'Control' 'waitForRos; roslaunch mrs_uav_general core.launch'
  'AutoStart' 'waitForRos; roslaunch mrs_uav_general automatic_start.launch
'
  'GoTo1' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-11.5, -35.3, 2.1, 0.2\]\";
'
  'GoTo2' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-10.7, -34.0, 2.1, -0.2\]\";
'
  'GoTo3' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-8.2, -32.0, 2.1, -1.25\]\";
'
  'GoTo4' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-7.5, -31.0, 2.1, -1.5\]\";
'
  'GoTo5' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-9.1, -33.0, 2.1, -0.6\]\";
'
  'GotoFINAL' 'history -s rosservice call /'"$UAV_NAME"'/control_manager/goto \"goal: \[-9.1, -33.0, 2.5, -0.6\]\";
'
  'Ampoule_arm' 'waitForRos; rosservice call /'"$UAV_NAME"'/ampoule_launcher/arm_launcher true'
  'Ampoule_fire' 'waitForRos; rosservice call /'"$UAV_NAME"'/ampoule_launcher/fire_launcher'
  'Lepton' 'cd ~/lepton; python3 record.py /dev/video0 test
'
  'Ampoule_launcher' 'waitForRos; roslaunch ampoule_launcher launcher.launch
'
  'Ampoule_serial' 'waitForRos; roslaunch ampoule_launcher serial.launch
'
  'slow_odom' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/slow_odom
'
  'odom_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/diagnostics
'
  'mavros_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/mavros_interface/diagnostics
'
  'kernel_log' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
)

init_window="Status"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=mav

# prefere the user-compiled tmux
if [ -f /usr/local/bin/tmux ]; then
  export TMUX_BIN=/usr/local/bin/tmux
else
  export TMUX_BIN=/usr/bin/tmux
fi

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then

  echo "The session already exists"
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

$TMUX_BIN -2 attach-session -t $SESSION_NAME

clear
