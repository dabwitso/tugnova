#!/bin/bash

AUTOWARE_BASE_DIR=/home/nvidia/Autoware
NODESHELL_DIR=$AUTOWARE_BASE_DIR/ros/nodeshell
TEMP_FILE=/home/nvidia/temp_file.log
TEMP_FOLDER=/home/nvidia/temp_folder
IS_GITLOOP="true"
IS_MASTER="true"


push_git () {
  # get commit details
  # prompt user for their name
  LOOPUSER="true"
  while $LOOPUSER;
  do
    USER_NAME=$(whiptail --title "GIT COMMIT INPUT INFO" --inputbox "Enter Your Name:"\
      --ok-button "Enter" --cancel-button "Abort" 0 0 3>&1 1>&2 2>&3)

    if [ $? -ne 0 ];
    then
      TERM=ansi whiptail --title "ABORTING" --infobox \
        "Aborting $TYPE save operation by user request..." 8 40

      sleep 5
      clear
      exit 1
    fi

    if [ -z "$USER_NAME" ];
    then
      TERM=ansi whiptail --title "ERROR" --infobox \
        "ERROR: Username NOT Entered.\nRetrying..." 8 40

      sleep 3
      clear
    else
      TERM=ansi whiptail --title "USERNAME SET" --infobox "Set username: $USER_NAME" 8 40

      sleep 2
      clear
      LOOPUSER="false"
    fi
  done
  # prompt user for commit message
  COMMIT_MESSAGE=$(whiptail --title "GIT COMMIT INPUT INFO" --ok-button "Enter"\
    --cancel-button "Skip" --inputbox "Enter Commit Message (optional):\
    \n[If No Commit Message Set, Default (Today's Date) is Used]" 0 0 3>&1 1>&2 2>&3)

  if [ $? -eq 0 ];
  then
    if [ -z $COMMIT_MESSAGE ];
    then
      COMMIT_MESSAGE=$(date)
    fi
  else
    COMMIT_MESSAGE=$(date)
  fi
  # prompt user for confirmation if this is a tested working version of autoware
  OK_CHECK=$(whiptail --title "GIT COMMIT INPUT INFO" --yes-button "Confirm"\
    --no-button "Not Tested" --yesno "Is this Version Tested OK?\
    \n[Tested version working without bugs]" 0 0 3>&1 1>&2 2>&3)

  if [ $? -eq 0 ];
  then
    OK_CHECK="[OK]"
  else
    OK_CHECK="[NG]"
  fi
  git add "$@"
  git commit -m "TESTED: $OK_CHECK, $USER_NAME -> $COMMIT_MESSAGE"

  whiptail --title "CONNECT REQUEST" --yesno "Do You Want to Push to Remote Git?"\
    0 0 --yes-button "Push" --no-button "Exit"

  if [ $? -ne 0 ];
  then
    TERM=ansi whiptail --title "STATUS" --infobox \
      "Save to Local Git Completed!\nExiting..." 8 40

    sleep 5
    clear
    exit 0
  fi

  TERM=ansi whiptail --title "GIT REMOTE OPERATION"\
    --infobox "Connecting to remote git..." 8 40

  sleep 4
  clear
  # start remote git push operation
  while [ $IS_GITLOOP == "true" ];
  do
    git pull && git push && IS_GITLOOP="false" || \
      whiptail --title "ERROR" --yesno "ERROR: Git Push Failed\nDo you want to retry?"\
      0 0 --yes-button "Retry" --no-button "Abort"

    PUSHSTATUS=$?
    # handle push failure
    if [[ $IS_GITLOOP == "true" ]];then
      if [[ $PUSHSTATUS -eq 0 ]];then
        TERM=ansi whiptail --title "GIT REMOTE OPERATION" --infobox "Retrying git push..." 8 40

        sleep 3
        clear
      elif [[ $PUSHSTATUS -eq 1 || $PUSHSTATUS -eq 255 ]];
      then
        TERM=ansi whiptail --title "ERROR" --infobox "Git push failed. Retry declined by user.\
          \nAborting Git Push to Remote Operation..\
          \nSaved to Local Git ONLY!" 12 40

        sleep 5
        clear
        IS_GITLOOP="false"
      fi
    fi
  done # end of git clone loop
}

isdirectory () {
  # check if file or directory
  if [ -d $1 ];
  then
    true
  elif [ -f $1 ];
  then
    false
  fi
}

reset_branch () {
  PATHS=($@)
  git status > $TEMP_FILE
  while read LINE;
  do
    case $LINE in
      *detached*) IS_MASTER="false"
        TERM=ansi whiptail --title "PROGRESS" --infobox \
          "Detached HEAD detected.\nConfiguring Checkout to master..." 8 40

        sleep 3
        clear
        ;;
    esac
    break
  done < $TEMP_FILE
  rm $TEMP_FILE
  if [ $IS_MASTER == "false" ];
  then
    if [ ! -d $TEMP_FOLDER ];
    then
      mkdir $TEMP_FOLDER
    else
      rm $TEMP_FOLDER/* 2>/dev/null
    fi
    # copy file to temp folder
    TERM=ansi whiptail --title "PROGRESS" --infobox \
      "Copying files to temp folder" 8 40

    sleep 3
    clear

    DIR_NAMES=()
    for path in "${PATHS[@]}";
    do
      if isdirectory $path;
      then
        name=$( rev <<< $(cut -d/ -f 2 <<< $( rev <<< $path )))
        DIR_NAMES+=($name "")
        cp -r $path $TEMP_FOLDER
        rm -r $path
      else
        name=$( rev <<< $(cut -d/ -f 1 <<< $( rev <<< $path )))
        DIR_NAMES+=($name "")
        cp $path $TEMP_FOLDER
        rm $path
      fi
    done
    # checkout master
    TERM=ansi whiptail --title "PROGRESS" --infobox \
      "Checkout HEAD to master" 8 40

    sleep 3
    clear
    git checkout -- $AUTOWARE_BASE_DIR 2>/dev/null
    git checkout master
    # copy files from temp folder
    TERM=ansi whiptail --title "PROGRESS" --infobox \
      "Copying From Temp Folder..." 8 40

    sleep 3
    clear
    COUNTER=0
    for file in "${DIR_NAMES[@]}";
    do
      if [ ! -z $file ];
      then
        cp -r $TEMP_FOLDER/$file "${PATHS[$COUNTER]}"
        COUNTER=$((COUNTER+1))
      fi
    done
  fi
}

save_files () {
  rm $TEMP_FILE >&/dev/null
  # print git status to temp file
  git status -s | cut -c 3- > $TEMP_FILE

  FILE_LIST=()
  while read FILE;
  do
    FILE_LIST+=("${FILE}" "" "OFF")
  done < $TEMP_FILE

  SELECTED_FILES=$(whiptail --title "COMMIT FILE OPTIONS" --checklist \
    "Select Files to Save to Git\nPress [ Space ] to Select:" 0 0 0 \
    "${FILE_LIST[@]}" --ok-button "Select" --cancel-button "Abort" 3>&1 1>&2 2>&3)

  if [ $? -ne 0 ];
  then
    TERM=ansi whiptail --title "ERROR" --infobox \
      "No File Selected to Save.\
      \nTerminating Operation and Exiting..." 8 40

    sleep 5
    clear
    exit 1
  fi
  ADD_FILES=()
  for i in ${SELECTED_FILES[@]};
  do
    i=${i#'"'}
    ADD_FILES+=("${i%'"'}")
  done

  # resolve detached HEAD cases
  reset_branch "${ADD_FILES[@]}"

  # save files to local and, if successful, to remote git repository
  push_git "${ADD_FILES[@]}"
}

save_files
