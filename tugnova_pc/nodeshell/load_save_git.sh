#!/bin/bash

AUTOWARE_BASE_DIR=/home/nvidia/Autoware
AUTOWARE_MAP_DIR=$AUTOWARE_BASE_DIR/map
AUTOWARE_ROUTE_DIR=$AUTOWARE_BASE_DIR/ROUTE
NODESHELL_DIR=$AUTOWARE_BASE_DIR/ros/nodeshell
GIT_DIR=/home/nvidia/gitlab
GIT_REMOTE_BASE_URI=https://tmc-droom-gitlab.com/kamigo/tugnova
IS_GITLOOP="true"
MODECHECK="true"
FILE_TYPE=$1
INITIAL_SETUP=$2
TEMP_MAP_LOG=$GIT_DIR/maps.log
DEFAULT_MAP=kamigo.pcd

${INITIAL_SETUP:=not_initial_setup} >&/dev/null

git_loop () {
  PUSHSTATUS=$1
  if [ $IS_GITLOOP == "true" ];then
    if [ $PUSHSTATUS -eq 0 ];then
      TERM=ansi whiptail --title "GIT STATUS" --infobox "Retrying Git Operation..." 8 40

      sleep 3
      clear -x
    elif [[ $PUSHSTATUS -eq 1 || $PUSHSTATUS -eq 255 ]];
    then
      if [ $IS_INITIAL_SETUP == "initial" ];
      then
        TERM=ansi whiptail --title "ERROR" --infobox \
          "Clone Failed. Retry Declined by User.\
          \nTerminating Operation and Exiting..." 8 40

        sleep 5
        clear -x
        exit 1
      else
        TERM=ansi whiptail --title "ERROR" --infobox \
          "Git Operation Failed. Retry Declined by User.\
          \nAborting Remote Clone Request..." 8 40

        sleep 5
        clear -x
        IS_GITLOOP="false"
      fi
    fi
  fi
}

pull_remote_git () {
  TYPE=$1
  # check if git folder exists
  if [ ! -d $GIT_DIR/$TYPE ]
  then
    TERM=ansi whiptail --title "REMOTE CONNECTION" --infobox \
      "WARNING: $GIT_DIR/$TYPE folder not found. Creating folder...\
      \nEnsure internet connectivity for connecting to remote git repo\
      \nAttempting remote repository clone..." 12 40

    sleep 5
    clear -x

    # create parent folder
    mkdir $GIT_DIR >&/dev/null
    cd $GIT_DIR

    # start git clone loop
    while [[ $IS_GITLOOP == "true" ]]
    do
      echo "Starting Git Clone Operation..."
      git clone $GIT_REMOTE_BASE_URI/$TYPE.git && IS_GITLOOP="false" || \
        whiptail --title "ERROR" --yesno \
        "Error: Git Clone Failed\nDo You Want to Retry?"\
        0 0 --yes-button "Retry" --no-button "Abort"

      # handle clone failure
      git_loop $?

    done # end of git clone loop
  else
    # pull remote repo files into existing folder
    cd $GIT_DIR/$TYPE
    while [[ $IS_GITLOOP == "true" ]]
    do
      echo "Starting Git Pull Operation..."
      git checkout -- .
      git checkout master && git pull && IS_GITLOOP="false" || \
        whiptail --title "ERROR" --yesno \
        "Error: Git Clone Failed\nDo You Want to Retry?"\
        0 0 --yes-button "Retry" --no-button "Abort"

      if [ $? -ne 0 ];
      then
        TERM=ansi whiptail --title "RESOLVE ERROR" --infobox \
          "Git Clone Failed, possible merge conflict occurred.\
          \nRun following commands to resolve merge conflict:\
          \n1. rm -rf ~/gitlab/$TYPE\
          \nNext, reload $TYPE files by running:
                  \n2. ./${NODESHELL_DIR}/map_route_save_load.sh"\
                    20 40

                  exit 1
      fi

      # handle clone failure
      git_loop $?
    done
  fi
}

push_remote_git () {
  TYPE=$1
  COMMIT_TYPE=$2
  cd $GIT_DIR/$TYPE

  # reset HEAD to master before adding new commit in case HEAD was in detached mode
  git checkout -- .
  git checkout master

  # copy files to local repository for saving
  if [ $TYPE == "routes" ];
  then
    rm *.csv
    cp $AUTOWARE_ROUTE_DIR/*.csv .
  else
    cp $AUTOWARE_MAP_DIR/*.pcd . >&/dev/null
  fi

  # get commit details
  LOOPUSER="true"
  while $LOOPUSER;
  do
    USER_NAME=$(whiptail --title "GIT COMMIT INPUT INFO" --inputbox "Enter Your Name:"\
      --ok-button "Enter" --cancel-button "Abort" 0 0 3>&1 1>&2 2>&3)

    if [ $? -ne 0 ];
    then
      TERM=ansi whiptail --title "ABORTING" --infobox \
        "Aborting $TYPE save operation by user request..." 8 40

      sleep 4
      clear -x
      exit 1
    fi

    if [ -z "$USER_NAME" ];
    then
      TERM=ansi whiptail --title "ERROR" --infobox \
        "ERROR: Username NOT Entered.\nRetrying..." 8 40

      sleep 3
      clear -x
    else
      TERM=ansi whiptail --title "USERNAME SET" --infobox "Set username: $USER_NAME" 8 40

      sleep 2
      clear -x
      LOOPUSER="false"
    fi
  done
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

  git add .
  git commit -m "Committer name: $USER_NAME -> $COMMIT_MESSAGE"
  if [ $COMMIT_TYPE == "remote" ];
  then
    while [ $IS_GITLOOP == "true" ];
    do
      git pull && git push && IS_GITLOOP="false" || \
        whiptail --title "ERROR" --yesno "ERROR: Git operation failed\nDo you want to retry?"\
        0 0 --yes-button "Retry" --no-button "Abort"

      PUSHSTATUS=$?
      # handle push failure
      if [[ $IS_GITLOOP == "true" ]];then
        if [[ $PUSHSTATUS -eq 0 ]];then
          TERM=ansi whiptail --title "GIT REMOTE OPERATION" --infobox "Retrying git operation..." 8 40

          sleep 3
          clear -x
        elif [[ $PUSHSTATUS -eq 1 || $PUSHSTATUS -eq 255 ]];
        then
          TERM=ansi whiptail --title "ERROR" --infobox "Git push failed. Retry declined by user.\
            \nAborting $TYPE git push operation..." 8 40
          git reset --mixed HEAD~1

          sleep 5
          clear -x
          IS_GITLOOP="false"
        fi
      fi
    done # end of git clone loop
  else
    TERM=ansi whiptail --title "GIT LOCAL OPERATION" --infobox \
      "File commit to local git history only, completed.\
      \n(This option is mostly for test files)" 8 40

    sleep 4
    clear -x
  fi
}

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
  TERM=ansi whiptail --title "GIT CHECKOUT" --infobox \
    "Switching Git HEAD to $COMMIT_CODE commit Version\
    \nWARNING: All uncommitted changes overwritten" 8 40

  sleep 3
  clear -x
  git checkout -- .
  git checkout $COMMIT_CODE
  rm git_history.log
}



while [ $MODECHECK == "true" ];
do
  # check if new tugnova setup operation
  if [ $INITIAL_SETUP == "initial" ];
  then
    MODE=1
    REMOTE_CONNECT="y"
  else
    # select operation type from menu
    MODE=$(whiptail --title "OPERATION TYPE" --menu \
      "Select Operation Type:" 0 0 2 \
      --ok-button "Select" --cancel-button "Abort"\
      "1. " "Load Files"\
      "2. " "Save Files"\
      3>&1 1>&2 2>&3
    )
    if [ $? -ne 0 ];
    then
      TERM=ansi whiptail --title "ABORTING" --infobox "Aborting by User Request..." 8 40

      sleep 2
      clear -x
      exit 1
    fi
    MODE=${MODE%'. '}
    #select whether to connect to remote git server or not
    whiptail --title "REMOTE GIT CONNECT" --yesno \
      "Do you want to connect to remote git repository?"\
      0 0 --yes-button "Connect" --no-button "Decline"

    USER_INPUT=$?
    if [ $USER_INPUT -eq 0 ];
    then
      REMOTE_CONNECT="y"
      TERM=ansi whiptail --title "REMOTE GIT CONNECT" --infobox \
        "Set to Connect to remote git repository..." 8 40

      sleep 2
      clear -x
    else
      REMOTE_CONNECT="n"
      TERM=ansi whiptail --title "REMOTE GIT CONNECT" --infobox \
        "Skipping remote connection..." 8 40

      sleep 2
      clear -x
    fi
  fi
  # Process operation types according to MODE selected
  if [ $MODE -eq 1 ];
  then
    if [ $FILE_TYPE == "maps" ];
    then
      # process remote git connection
      if [ $REMOTE_CONNECT == "y" ];
      then
        pull_remote_git $FILE_TYPE
      fi

      TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
        "Starting to load file(s) into $AUTOWARE_MAP_DIR..." 8 40

      sleep 3
      clear -x
      # check if source files exist
      if [ ! -d $GIT_DIR/$FILE_TYPE ];
      then
        TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
          "Error: Cannot load files for non-existent directory $GIT_DIR/$FILE_TYPE\
          \nTerminating..." 8 40

        sleep 5
        clear -x
        exit 1
      fi
      cd $GIT_DIR/$FILE_TYPE

      # choose version to load from git history list
      show_git_history

      MENU_OPTIONS=()
      ls >> $TEMP_MAP_LOG

      while read LINE;
      do
        MENU_OPTIONS+=(${LINE%.pcd} "   Map file ${LINE}")
      done < $TEMP_MAP_LOG

      MAP_FILE=$(whiptail --title "LIST OF AVAILABLE MAPS" --menu "Select Map File:"\
        0 0 0 "${MENU_OPTIONS[@]}" --ok-button "Select" --cancel-button "Abort" 3>&1 1>&2 2>&3)

      if [ $? -eq 0 ];
      then
        MAP_FILE+=.pcd
      else
        MAP_FILE=$DEFAULT_MAP
        TERM=ansi whiptail --title "ABORTING SELECTION" --infobox \
          "Aborting user settings. Falling back to Default map: $MAP_FILE" 8 40

        sleep 3
        clear -x
      fi
      rm $TEMP_MAP_LOG

      rm $AUTOWARE_MAP_DIR/*.pcd >&/dev/null
      cp $MAP_FILE $AUTOWARE_MAP_DIR/

    elif [ $FILE_TYPE == "routes" ];
    then
      # process remote git connection
      if [ $REMOTE_CONNECT == "y" ];
      then
        pull_remote_git $FILE_TYPE
      fi

      # check if folder trying to load files from exists
      if [ ! -d $GIT_DIR/$FILE_TYPE ];
      then
        TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
          "Error: Cannot load files for non-existent directory $GIT_DIR/$FILE_TYPE\
          \nTerminating..." 8 40

        sleep 5
        clear -x
        exit 1
      fi

      TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
        "Starting to load file(s) into $AUTOWARE_ROUTE_DIR..." 8 40

      sleep 3
      clear -x

      whiptail --title "LOAD TEST FILES" --yesno \
        "Do you want to load test route files?\
        \n[Files not committed to local git history, but only saved to folder]"\
        --yes-button "LOAD_TEST" --no-button "LOAD_GIT" 0 0

      ISTEST=$?
      if [ $ISTEST -eq 0 ];
      then
        bash $NODESHELL_DIR/load_routes.sh
      elif [[ $ISTEST -eq 1 || $ISTEST -eq 255 ]];
      then
        cd $GIT_DIR/$FILE_TYPE

        show_git_history

        rm $AUTOWARE_BASE_DIR/ROUTE/*.csv >&/dev/null
        cp $GIT_DIR/$FILE_TYPE/*.csv $AUTOWARE_BASE_DIR/ROUTE
      fi
    fi
    TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
      "Loaded $FILE_TYPE into Autoware successfully!" 8 40

    sleep 3
    clear -x
    MODECHECK="false"

  elif [ $MODE == 2 ];
  then
    TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
      "Starting to save $FILE_TYPE into local gitlab folder..." 8 40

    sleep 3
    clear -x
    # Check file(s) type
    if [ $FILE_TYPE == "maps" ];
    then
      if [ $REMOTE_CONNECT == "y" ];
      then
        push_remote_git $FILE_TYPE "remote"
      elif [ $REMOTE_CONNECT == "n" ];
      then
        whiptail --title "LOCAL GIT" --yesno \
          "Do you want to commit changes to local git history instead?\
          \n(Not recommended for test files)" --yes-button "Commit" --no-button "Skip" 0 0

        USER_INPUT=$?
        if [ $USER_INPUT -eq 0 ];
        then
          push_remote_git $FILE_TYPE "local"
        elif [[ $USER_INPUT -eq 1 || $USER_INPUT -eq 255 ]];
        then
          cp $AUTOWARE_ROUTE_DIR/*.pcd $GIT_DIR/$FILE_TYPE/
        fi
      fi
    elif [ $FILE_TYPE == "routes" ];
    then
      if [ $REMOTE_CONNECT == "y" ];
      then
        push_remote_git $FILE_TYPE "remote"
      elif [ $REMOTE_CONNECT == "n" ];
      then
        rm $GIT_DIR/$FILE_TYPE/*.csv >&/dev/null
        cp $AUTOWARE_ROUTE_DIR/*.csv $GIT_DIR/$FILE_TYPE/
      fi
    fi
    TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
      "Save $FILE_TYPE into relatively static folder successful!" 8 40

    sleep 3
    clear -x
    MODECHECK="false"
  else
    TERM=ansi whiptail --title "OPERATION STATUS" --infobox \
      "Invalid option number selected.\nPress [Ctrl + C] to terminate, or continue retry" 8 40

    sleep 5
    clear -x
  fi
done


