#!/bin/bash

CART_ID=$1

rostopic pub -1 /cart_id std_msgs/String "data: ${CART_ID}"
echo "Cart_ID: ${CART_ID} published to drive_controller node"

