#!/bin/bash

NODESHELL_DIR=/home/nvidia/Autoware/ros/nodeshell


OPTION=$(whiptail --title "OPERATION MODE" --menu \
  "Choose an option" 0 0 0 \
  --ok-button "Select" --cancel-button "Abort" \
  "1. " "Map" \
  "2. " "Route" \
  3>&1 1>&2 2>&3
)

# check if Abort or ESC entered by user
if [[ $? -ne 0 ]];
then
  TERM=ansi whiptail --title "ABORTING" --infobox "Aborting by User Request..." 8 40

  sleep 2
  clear
  exit 1
fi

# process selection
OPTION=${OPTION%'. '}
if [ $OPTION == 1 ];
then
  TERM=ansi whiptail --title "CONFIRMATION" --infobox "Starting to process map files..." 8 40

  sleep 2
  clear

  bash $NODESHELL_DIR/git_map.sh
  MODECHECK="false"
else
  TERM=ansi whiptail --title "CONFIRMATION" --infobox "Starting to process route files..." 8 40

  sleep 2
  clear

  bash $NODESHELL_DIR/git_routes.sh
  MODECHECK="false"
fi
