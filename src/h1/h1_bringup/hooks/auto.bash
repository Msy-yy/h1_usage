#!/bin/bash

CYAN='\033[0;36m'

export H1_NS="h1_unit_238";
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
export CYCLONEDDS_URI=/opt/mybotshop/src/mybotshop/h1_bringup/config/def_cyclone.xml;

echo -e "${CYAN}MYBOTSHOP H1 Environment Setup Complete"