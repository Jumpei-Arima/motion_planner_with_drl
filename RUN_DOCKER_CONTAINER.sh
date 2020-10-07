#!/bin/bash

# Set the Docker container name from a project name (first argument).
PROJECT=arima
CONTAINER="${PROJECT}_mpdrl_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Run the Docker container in the background.
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml up -d

# Display GUI through X Server by granting full access to any external client.
xhost +

# Enter the Docker container with a Bash shell (with or without a custom 'roslaunch' command).
case "$1" in
  ( "" )
  docker exec -i -t ${CONTAINER} bash
  ;;
  ( ".launch" | \
    "weblab_real_default.launch" | \
    "weblab_rviz_default.launch")
  docker exec -i -t ${CONTAINER} bash -i -c "roslaunch mpdrl_ros $1"
  ;;
  ( * )
  echo "Failed to enter the Docker container '${CONTAINER}': '$1' is not a valid argument value."
  ;;
esac
