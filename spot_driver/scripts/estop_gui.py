#!/bin/bash

RED='\033[0;31m'
NC='\033[0m' # No Color

function print_error_and_exit() {
    printf "${RED}$1${NC}\n"
    exit 1
}

if [ $# -lt 1 ]; then
    print_error_and_exit "You must specify the robot name"
fi

echo 'Starting estop no gui'
source /bdai/leash_spot.bash
leash_spot $1
if [ $? -ne 0 ]; then
    exit $?
fi
python -m spot_driver.estop_gui
