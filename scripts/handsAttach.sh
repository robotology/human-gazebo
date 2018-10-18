usage() {
cat << EOF
***************************************************************************************
ATTACH SCRIPT FOR PHRI SIMULATION EXPERIMENT
Author:  Yeshasvi Tirupachuri   <yeshasvi.tirupachuri@iit.it>
This script takes name of the human model and attaches the hands of the human to iCub
USAGE:
        $0 options
***************************************************************************************
OPTIONS: <function specifier>, human_model_name
***************************************************************************************
EXAMPLE USAGE: ./attach attach sub0   -------> Attaching the hands of the human and robot
EXAMPLE USAGE: ./attach detach sub0   -------> Detaching the hands of the human and robot
EXAMPLE USAGE: ./attach attach iCub_0 -------> Attaching the hands of two iCub robots
EXAMPLE USAGE: ./attach detach iCub_0 -------> Detaching the hands of two iCub robots
***************************************************************************************
EOF
}

attach_iCub_left() {
  echo "attachUnscoped icubSim l_hand ${HUMAN} r_hand" | yarp rpc //linkattacher/rpc:i
}

attach_iCub_right() {
  echo "attachUnscoped icubSim r_hand ${HUMAN} l_hand" | yarp rpc //linkattacher/rpc:i
}

attach_human_left() {
  echo "attachUnscoped iCub l_hand ${HUMAN} RightHand" | yarp rpc /${HUMAN}/linkattacher/rpc:i
}

attach_human_right() {
  echo "attachUnscoped iCub r_hand ${HUMAN} LeftHand" | yarp rpc /${HUMAN}/linkattacher/rpc:i
}

detach_iCub_left() {
  echo "detachUnscoped icubSim l_hand" | yarp rpc //linkattacher/rpc:i
}

detach_iCub_right() {
  echo "detachUnscoped icubSim r_hand" | yarp rpc //linkattacher/rpc:i
}

detach_human_left() {
  echo "detachUnscoped iCub l_hand" | yarp rpc /${HUMAN}/linkattacher/rpc:i
}

detach_human_right() {
  echo "detachUnscoped iCub r_hand" | yarp rpc /${HUMAN}/linkattacher/rpc:i
}
################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
if [[ $# -eq 0 || $# -lt 2 ]] ; then
    echo "Error in options passed!"
    echo ""
    usage
    exit 1
fi

FUNCTION=$1
HUMAN=$2

if [[ ${FUNCTION} == "attach" ]] ; then
  if [[ ${HUMAN} == "iCub_0" ]] ; then
    attach_iCub_left
    attach_iCub_right
  fi
  if [[ ${HUMAN} == "sub0" ]] ; then
    attach_human_left
    attach_human_right
  fi
fi

if [[ ${FUNCTION} == "detach" ]] ; then
  if [[ ${HUMAN} == "iCub_0" ]] ; then
    detach_iCub_left
    detach_iCub_right
  fi
  if [[ ${HUMAN} == "sub0" ]] ; then
    detach_human_left
    detach_human_right
  fi
fi

exit 0
