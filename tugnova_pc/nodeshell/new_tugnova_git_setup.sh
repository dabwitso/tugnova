#!/bin/bash

AUTOWARE_BASE_DIR=/home/nvidia/Autoware
AUTOWARE_ROS_DIR=/$AUTOWARE_BASE_DIR/ros
NODESHELL_DIR=$AUTOWARE_ROS_DIR/nodeshell
HOME_DIR=/home/nvidia
GIT_DIR=$HOME_DIR/tugnova_vecow_pc
GIT_REPO_NAME=tugnova_vecow_pc
GIT_REMOTE_URI=https://tmc-droom-gitlab.com/kamigo/tugnova/$GIT_REPO_NAME.git

# parameter for looping git clone in case of failure
IS_GITLOOP="true"

cd $HOME_DIR

TERM=ansi whiptail --title "AUTOWARE SETUP" --infobox \
"Starting New Tugnova Setup..." 8 40

sleep 3
clear -x

TERM=ansi whiptail --title "AUTOWARE SETUP" --infobox \
 "Starting to pull file from remote git repository:\
 \n\n$GIT_REMOTE_URI\
 \n\nEnsure you have internet access before continuing" 8 40

sleep 5
clear -x

whiptail --title "INTERNET CONNECTIVITY"\
 --yesno "Do you have internet access?" 0 0 \
 --yes-button "Continue" --no-button "Abort"

# handle no input case. Default to yes
if [ $? -eq 0 ];
then
  TERM=ansi whiptail --title "INTERNET CONNECTIVITY"\
   --infobox "Connecting to internet..." 8 40

  sleep 3
  clear -x
else
  TERM=ansi whiptail --title "SETUP ERROR"\
   --infobox "No internet access option selected.\nTerminating setup..." 8 40

  sleep 5
  clear -x
  exit 1
fi

# start git clone loop
while [ $IS_GITLOOP == "true" ]
do
  git clone $GIT_REMOTE_URI >&/dev/null && IS_GITLOOP="false" || \
    whiptail --title "ERROR" --yesno "ERROR: Git clone failed\
    \nDo you want to retry?" 0 0 --yes-button "Retry" --no-button "Abort"

  CLONESTATUS=$?

  # handle clone failure
  if [ $IS_GITLOOP == "true" ];then
    if [ $CLONESTATUS -eq 0 ];then
      TERM=ansi whiptail --title "GIT CLONE" --infobox "Retrying git clone..." 8 40

      sleep 3
      clear -x
    else
      TERM=ansi whiptail --title "ERROR" --infobox \
      "Clone failed. Retry declined by user.\nTerminating setup..." 8 40

      sleep 5
      clear -x
      exit 1
    fi
  fi
done # end of git clone loop

# start folder configuring
rm Autoware
mv $GIT_REPO_NAME Autoware_1.15
ln -s Autoware_1.15 Autoware

# configure git route, param backup & map folder
chmod +x $NODESHELL_DIR/*.sh
bash $NODESHELL_DIR/create_param_static_folder.sh

mkdir $AUTOWARE_BASE_DIR/ROUTE >&/dev/null
bash $NODESHELL_DIR/git_routes.sh initial

mkdir $AUTOWARE_BASE_DIR/map >&/dev/null
bash $NODESHELL_DIR/git_map.sh initial

# compile autoware and hmi_tugnova_msgs git repo
bash $NODESHELL_DIR/autoware_full_compile.sh initial

whiptail --title "CONFIGURATION" --yesno "Do You Want to Set ROS_MASTER_URI in .bashrc file?"\
--yes-button "Set" --no-button "Skip" 0 0

if [ $? -eq 0 ];
then
  TERM=ansi whiptail --title "CONFIGURATION" --infobox \
   "Configuring ROS_MASTER_URI & ROS_IP settings in\
   \n/home/nvidia/.bashrc file" 8 40

  sleep 3
  clear -x

  echo "export ROS_MASTER_URI=http://192.168.0.37:11311" >> /home/nvidia/.bashrc
  echo "export ROS_IP=192.168.0.37" >> /home/nvidia/.bashrc
fi

whiptail --title "FINAL MANUAL SETUP" --textbox --scrolltext $NODESHELL_DIR/setup_instructions.txt 0 0

