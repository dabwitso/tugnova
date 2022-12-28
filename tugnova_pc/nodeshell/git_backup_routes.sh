#!/bin/bash

GIT_DIR=/home/nvidia/gitlab
AUTOWARE_ROUTE_DIR=/home/nvidia/Autoware/ROUTE
IS_GIT_LOOP="true"

TODAY_DATE=$(date)


ACTUAL_PWD="Freedom!1"

read -s -p "Enter password: " USER_PWD
echo ""


if [[ $ACTUAL_PWD == $USER_PWD ]];
then
  read -p "Enter your name: " USER_NAME
  if [[ ! -d $AUTOWARE_ROUTE_DIR ]];
  then
    echo "
    Error: Backup source directory $AUTOWARE_ROUTE_DIR not found. Aborting..."
    sleep 3
    exit 1
  fi

  echo "starting route backup..."

  if [[ ! -d $GIT_DIR/routes ]];
  then
    echo "$GIT_DIR/routes folder not found. Creating..."
    mkdir -p $GIT_DIR/routes >&/dev/null
  else
    rm $GIT/routes/*.csv >&/dev/null
  fi
  cp $AUTOWARE_ROUTE_DIR/*.csv $GIT_DIR/routes
  cd $GIT_DIR/routes
  git add .
  git commit -m "$USER_NAME: $TODAY_DATE"

  echo "Successfully copied files from $AUTOWARE_ROUTE_DIR to secret git backup folder"
  if [[ $IS_GIT_LOOP == "true" ]];
  then
    read -p "Do you want to push these changes to remote git repo? [y/n]: " DOPUSH
    if [[ $DOPUSH == "y" || $DOPUSH == "yes" ]];
    then
      echo "
      First pulling remote repo changes before committing local changes"
      git pull

      echo "Pushing local changes to remote repo..."
      git push
      read -p "If push not successful, do you want to retry git push? [y/n]: " RETRY
      if [[ $RETRY == "n" || $RETRY == "no" ]];
      then
        IS_GIT_LOOP="false"
      fi
    else
      IS_GIT_LOOP="false"
    fi
  fi

  echo "

  Process completed!"

else
  echo "
  Error: Wrong password. Terminating..."
  sleep 3
  exit 1
fi
