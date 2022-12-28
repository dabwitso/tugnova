#!/bin/bash
AUTOWARE_BASE_DIR=/home/nvidia/Autoware
AUTOWARE_ROS_DIR=$AUTOWARE_BASE_DIR/ros
GIT_DIR=/home/nvidia/gitlab
IS_GITLOOP="true"
REMOTE_GIT_REPO=https://tmc-droom-gitlab.com/kamigo/tugnova/hmi_tugnova_msgs.git
REMOTE_CONNECT="n"

cd $AUTOWARE_ROS_DIR

# set this parameter if run within another script in order to skip some parts
while getopts "s:h" opt;
do
  case ${opt} in
    h)
      echo "Usage: ./$0 [option...]"
      echo " -h help"
      echo " -s [optional arg: [slave] if this script is invoked within another script]"
      exit 0
      ;;
    s)
      IS_SLAVE_SCRIPT=${OPTARG};;
    /?)
      echo "Invalid option ${OPTARG}. Check usage below:"
      ./build_hmi_tugnova_msgs.sh -h
      exit 1
      ;;
  esac
done

# check if run within another script
if [[ -z $IS_SLAVE_SCRIPT ]];
then
  echo -e "\nINFO: No argument passed. Default to standalone mode\n"
  whiptail --title "REMOTE CONNECT" --yesno \
    "Do You Want to Connect to Remote Git Repo?"\
    0 0 --yes-button "Connect" --no-button "Decline"

  if [ $? -eq 0 ];
  then
    REMOTE_CONNECT="y"
  fi
  IS_SLAVE_SCRIPT="standalone"
else
  IS_SLAVE_SCRIPT="slave"
fi

copy_files() {
  if [ -d "$AUTOWARE_ROS_DIR/src/msgs/communication_msgs" ];
  then
    rm -r $AUTOWARE_ROS_DIR/src/msgs/communication_msgs
  fi
  cp -r $GIT_DIR/hmi_tugnova_msgs/communication_msgs $AUTOWARE_ROS_DIR/src/msgs/
}

reload_autoware() {
  # Compile messages and re-launch Autoware
  TERM=ansi whiptail --title "AUTOWARE MESSAGE COMPILE"\
    --infobox "Starting to compile and build messages...\
    \nWARNING: Closing Running Autoware Programs..." 8 40

  sleep 5
  clear

  bash ${AUTOWARE_ROS_DIR}/nodeshell/ExitAutoware.sh

  cd ${AUTOWARE_ROS_DIR}

  source /opt/ros/kinetic/setup.bash

  catkin_make --pkg communication_msgs


  TERM=ansi whiptail --title "RELAUNCH" --infobox "Re-launching Autoware" 8 40

  sleep 3
  clear

  cd ${AUTOWARE_BASE_DIR}

  bash ${AUTOWARE_BASE_DIR}/api/run_apiserver_ui.sh
}

pull_remote_git() {
  TERM=ansi whiptail --title "GIT CLONE INFO"\
    --infobox "Attempting remote git clone..." 8 40

  sleep 3
  clear
  mkdir $GIT_DIR >&/dev/null
  cd $GIT_DIR
  # start git clone loop
  while [ $IS_GITLOOP == "true" ]
  do
    if [[ $IS_SLAVE_SCRIPT == "slave" ]];
    then
      if [ ! -d "$GIT_DIR/hmi_tugnova_msgs" ];
      then
        git clone $REMOTE_GIT_REPO >&/dev/null && IS_GITLOOP="false" ||\
          whiptail --title "ERROR" --yesno "Error: Git clone failed\
          \nDo you want to retry?" 0 0 --yes-button "Retry" --no-button "Abort"

      else
        TERM=ansi whiptail --title "GIT INFO"\
          --infobox "$GIT_DIR/hmi_tugnova_msgs Folder Exists. Skipping git clone..."\
          8 40

        sleep 4
        clear
        IS_GITLOOP="false"
      fi
    else
      git checkout -- . && git checkout master && git pull >&/dev/null && IS_GITLOOP="false" ||\
        whiptail --title "ERROR" --yesno "Error: Git pull failed\
        \nDo you want to retry?" 0 0 --yes-button "Retry" --no-button "Abort"
    fi

    CLONESTATUS=$?
    echo $CLONESTATUS

    if [ $IS_GITLOOP == "true" ];then
      if [ $CLONESTATUS -eq 0 ];
      then
        TERM=ansi whiptail --title "GIT INFO"\
          --infobox "Retrying remote git operation..." 8 40

        sleep 3
        clear
      else
        if [[ $IS_SLAVE_SCRIPT == "slave" ]];
        then
          TERM=ansi whiptail --title "ERROR"\
            --infobox "Clone failed. Retry Declined By User.\
            \n\nTerminating operation and exiting..." 10 40

          sleep 5
          clear
          exit 1
        else
          TERM=ansi whiptail --title "ERROR"\
            --infobox "Pull failed. Retry Declined By User.\
            \n\nAborting remote pull request..." 10 40

          sleep 5
          clear
          IS_GITLOOP="false"
        fi
      fi
    fi
  done # end of git clone loop

}

if [ $IS_SLAVE_SCRIPT == "slave" ];
then
  pull_remote_git
  copy_files
  TERM=ansi whiptail --title "UPDATE STATUS"\
    --infobox "Done updating hmi_tugnova_msgs files" 8 40

  sleep 3
  clear
else
  if [ ! -d "$GIT_DIR/hmi_tugnova_msgs" ];
  then
    if [ $REMOTE_CONNECT != "y" ];
    then
      whiptail --title "WARNING" --yesno \
        "WARNING: $GIT_DIR/hmi_tugnova_msgs folder not found.\
        \nDo you want to attempt cloning from remote server?"\
        --yes-button "Clone" --no-button "Abort" 0 0

      USER_INPUT=$?
      if [ $USER_INPUT -ne 0 ];
      then
        TERM=ansi whiptail --title "ABORT" --infobox \
          "ERROR: Git local folder not found\nAborting message update..." 8 40

        sleep 5
        clear
        exit 1
      fi
    fi
    IS_SLAVE_SCRIPT="slave"
    pull_remote_git
  else
    if [ $REMOTE_CONNECT == "y" ];
    then
      cd $GIT_DIR/hmi_tugnova_msgs
      git checkout -- .
      git checkout master
      git pull
    fi
  fi
  copy_files

  TERM=ansi whiptail --title "UPDATE STATUS" --infobox \
    "Done updating hmi_tugnova_msgs files" 8 40

  sleep 3
  clear
  reload_autoware
fi
