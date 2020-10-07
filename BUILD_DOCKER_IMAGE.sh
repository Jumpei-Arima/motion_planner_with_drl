#!/bin/bash

PROJECT=arima
echo "$0: PROJECT=${PROJECT}"

docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml build
