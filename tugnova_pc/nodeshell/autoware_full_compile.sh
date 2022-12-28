#!/bin/bash

AUTOWARE_ROS_DIR=/home/nvidia/Autoware/ros
NODESHELL_DIR=$AUTOWARE_ROS_DIR/nodeshell
FINAL_MESSAGE_SRC=$AUTOWARE_ROS_DIR/nodeshell/auto_compile_message.txt
BUILD_TEMP_LOG=$AUTOWARE_ROS_DIR/build.log
IS_INITIAL_SETUP=$1
IS_GITLOOP="true"
IS_GIT_PULLED="false"

cd $AUTOWARE_ROS_DIR

source /opt/ros/kinetic/setup.bash

${IS_INITIAL_SETUP:=not_initial_setup} >&/dev/null

show_git_history () {
  # print git commit history to log file
  git log --all --pretty=format:'%h [%cd] %s' --date=short >> git_history.log

  MENU_OPTIONS=()
  while read HASH COMMENT;
  do
    MENU_OPTIONS+=($HASH "${COMMENT}")
  done < git_history.log

  COMMIT_CODE=$(whiptail --title "GIT COMMIT HISTORY" --menu \
  "Select Git Commit Version to Activate:\n(Press [ ESC ] to select default latest commit version)"\
  0 0 0 "${MENU_OPTIONS[@]}" --ok-button "Select" --cancel-button "Default" 3>&1 1>&2 2>&3)

  #check if no commit code input, in case user chose default latest version
  if [ $? -ne 0 ];
  then
    TERM=ansi whiptail --title "CONFIRMATION" --infobox "Default Latest Commit Version Set" 8 40

    sleep 3
    clear -x
    COMMIT_CODE="master"
  fi

  #Move HEAD to selected commit point
  TERM=ansi whiptail --title "GIT CHECKOUT INFO" --infobox \
  "Switching Git HEAD to $COMMIT_CODE commit Version\
  \nWARNING: All uncommitted changes overwritten" 8 40

  sleep 3
  clear -x
  git checkout -- .
  git checkout $COMMIT_CODE
  rm git_history.log
}



pull_git_remote() {
  # start git clone loop
  while [ $IS_GITLOOP == "true" ]
  do
    git checkout -- . && git checkout master && git pull >&/dev/null && IS_GITLOOP="false" \
      && IS_GIT_PULLED="true" || whiptail --title "ERROR" --yesno \
      "ERROR: Git pull failed\nDo you want to retry?"\
      0 0 --yes-button "Retry" --no-button "Abort"

    CLONESTATUS=$?
    # handle clone failure
    if [ $IS_GITLOOP == "true" ];then
      if [ $CLONESTATUS -eq 0 ];then
        TERM=ansi whiptail --title "GIT PULL INFO" --infobox "Retrying git pull..." 8 40

        sleep 3
        clear -x
      else
        TERM=ansi whiptail --title "ERROR" --infobox \
        "Git pull failed. Retry declined by user.\nAborting remote pull request..." 8 40

        sleep 5
        clear -x
        IS_GITLOOP="false"
      fi
    fi
  done # end of git clone loop

}

progress_bar() {
  sleep 5

  while read LINE;
  do
    sleep 0.05
    PER=$(echo $LINE | grep "[0-9]%")
    MSG=$(echo $LINE | grep "Built target")
    if [[ -n $MSG ]];
    then
      MESSAGE=${MSG:6}
    else
      MESSAGE="Compiling & Linking Libraries..."
    fi
    if [[ -n $PER ]];
    then
      PERCENT=${PER:1:3}
    fi
    echo -e "XXX\n${PERCENT%'%'}\n$MESSAGE\nXXX"
  done < $BUILD_TEMP_LOG | whiptail --title "AUTOWARE COMPILE PROGRESS"\
    --gauge "Compiling Packages..." 10 50 1

}

if [ $IS_INITIAL_SETUP == "not_initial_setup" ];
then
  whiptail --title "AUTOWARE COMPILE" --yesno \
  "This script will build all autoware packages.\
  \nDo you want to continue?" --yes-button "Continue" --no-button "Abort" 0 0

  if [ $? -eq 0 ];
  then
    TERM=ansi whiptail --title "CONFIRMATION" --infobox \
    "Continuing to build & compile Autoware packages..." 8 40

    sleep 3
    clear -x
    # option to pull latest version from remote repo
    whiptail --title "GIT REMOTE CONNECT" --yesno \
    "Do you want to connect to remote git repo.\
    \nSelect yes if you want to pull remote gitlab files"\
    --yes-button "Connect" --no-button "Decline" 0 0

    if [ $? -eq 0 ];
    then
      pull_git_remote
    fi
    # show list of all commits to select from
    show_git_history

    # configure lidar id params into Autoware
    TERM=ansi whiptail --title "CONFIGURATION" --infobox "Configuring lidar ID params..." 8 40

    sleep 3
    clear -x
    bash $NODESHELL_DIR/set_lidar_params.sh
  else
    TERM=ansi whiptail --title "ABORTING" --infobox "Terminating by user request..." 8 40

    sleep 5
    clear -x
    exit 1
  fi
fi

# update hmi_tugnova messages
#TERM=ansi whiptail --title "MESSAGE UPDATE" --infobox \
#"Updating tugnova-hmi server shared messages from git folder..." 8 40
#
#sleep 3
#clear -x
#
#bash $AUTOWARE_ROS_DIR/nodeshell/build_hmi_tugnova_msgs.sh -s slave

# create build and devel backups of previous compile, in case need to use them
TERM=ansi whiptail --title "BACKUP" --infobox \
"Creating devel and build folder backups..." 8 40

sleep 3
clear -x

if [[ -d $AUTOWARE_ROS_DIR/build && -d $AUTOWARE_ROS_DIR/devel ]];
then
  if [[ -d $AUTOWARE_ROS_DIR/build_previous || -d $AUTOWARE_ROS_DIR/devel_previous ]];
  then
    rm -rf build_previous >&/dev/null
    rm -rf devel_previous >&/dev/null
  fi
  mv build build_previous
  mv devel devel_previous
fi

# Start compiling Autoware
TERM=ansi whiptail --title "AUTOWARE BUILD & COMPILE" --infobox \
"Starting to Build & Compile Autoware Messages\nPlease wait..." 12 40

sleep 4
clear -x

#spawn compile window in quiet mode.
#Generate build.log, which is used by whiptail gauge as a way to track progress
catkin_make --pkg communication_msgs udp_msgs carctl_msgs autoware_msgs autoware_can_msgs autoware_system_msgs dbw_mkz_msgs autoware_config_msgs >> $BUILD_TEMP_LOG

# compile all packages
TERM=ansi whiptail --title "AUTOWARE BUILD & COMPILE" --infobox "Starting to Compile All Packages..." 8 40

sleep 4
clear -x
rm $BUILD_TEMP_LOG
cd $AUTOWARE_ROS_DIR && catkin_make -DCATKIN_BlACKLIST_PACKAGES="communication_msgs;udp_msgs;carctl_msgs;autoware_msgs;autoware_can_msgs;autoware_system_msgs;dbw_mkz_msgs;autoware_config_msgs" >> $BUILD_TEMP_LOG

if [ $? -ne 0 ];
then
  TERM=ansi whiptail --title "ABORTING" --infobox "All Package Compile Failed.\
  \nCheck $BUILD_TEMP_LOG for details. \nTerminating...\nPress [ Ctrl + C ] to exit this notice" 14 40

  sleep 100
  clear -x
  exit 1
fi

#Generate whiptail progress gauge
#progress_bar

if [ $IS_INITIAL_SETUP == "not_initial_setup" ];
then
  whiptail --title "COMPILE COMPLETED" --textbox --scrolltext $FINAL_MESSAGE_SRC 0 0
fi
