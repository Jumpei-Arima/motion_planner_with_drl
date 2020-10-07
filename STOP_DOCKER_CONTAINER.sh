#!/bin/bash

# Set the Docker container name from a project name (first argument).
PROJECT=arima
CONTAINER="${PROJECT}_mpdrl_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

docker container stop ${CONTAINER}
docker container rm ${CONTAINER}
