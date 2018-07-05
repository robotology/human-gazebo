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
EXAMPLE USAGE: ./attach attach sub0 -------> Attaching the hands of the human and robot
EXAMPLE USAGE: ./attach detach sub0 -------> Detaching the hands of the human and robot
***************************************************************************************
EOF
}

attach_left() {
  echo "attachUnscoped iCub l_hand ${HUMAN} RightHand" | yarp rpc /${HUMAN}/objectattacher/rpc:i
}

attach_right() {
  echo "attachUnscoped iCub r_hand ${HUMAN} LeftHand" | yarp rpc /${HUMAN}/objectattacher/rpc:i
}

detach_left() {
  echo "detachUnscoped iCub l_hand" | yarp rpc /${HUMAN}/objectattacher/rpc:i
}

detach_right() {
  echo "detachUnscoped iCub r_hand" | yarp rpc /${HUMAN}/objectattacher/rpc:i
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
  attach_left
  attach_right
fi

if [[ ${FUNCTION} == "detach" ]] ; then
  detach_left
  detach_right
fi

exit 0
