#!/usr/bin/env bash

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 

function install_python () {
    ${BLUE} "Installing dependency."${NC}
    sudo apt-get install python3-pip
    pip3 install cython
}

current_directory=$PWD

echo -e "${DGREEN}-------------------------------------------------------------------------------"
echo -e "  __  ____     ______   ____ _______ _____ _    _  ____  _____           _____ "
echo -e " |  \/  \ \   / /  _ \ / __ \__   __/ ____| |  | |/ __ \|  __ \         / ____|"
echo -e " | \  / |\ \_/ /| |_) | |  | | | | | (___ | |__| | |  | | |__) |  _   _| |  __ "
echo -e " | |\/| | \   / |  _ <| |  | | | |  \___ \|  __  | |  | |  ___/  | | | | | |_ |"
echo -e " | |  | |  | |  | |_) | |__| | | |  ____) | |  | | |__| | |      | |_| | |__| |"
echo -e " |_|  |_|  |_|  |____/ \____/  |_| |_____/|_|  |_|\____/|_|       \__,_|\_____|"
echo -e ""                                                                                                                                         
echo -e "-------------------------------------------------------------------------------"
echo -e "Python Binary Generator"                                                                                                                                         
echo -e "-------------------------------------------------------------------------------${NC}"

# Dependency installation
# install_python

python3 ignore_compile.py build_ext --inplace

rm -rf build *.c