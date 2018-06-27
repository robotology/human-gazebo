#!/bin/bash

#################################################################################################
# "tune" FUNCTION: Tunes each joint with randomly chosen PID values from the initial values     #
#################################################################################################
tune() {

if [[ "${FUNCNAME[1]}" == "all_joints" ]] ; then
    CUR_DIR=$(echo "${PWD##*/}")
    if [[ "${CUR_DIR}" == 0 || "${CUR_DIR}" == 1 || "${CUR_DIR}" == 2 ]] ; then
        cd ../../
    fi
    echo "$1"
    PART=$1
    JOINT=1
fi

if [[ ! -d "${PART}" ]] ; then
    echo "Making ${PART} directory"
    mkdir ${PART}
fi

cd ${PART}

if [[ ! -d "${JOINT}" ]] ; then
    echo "Making ${JOINT} directory"
    mkdir ${JOINT}
fi

cd ${JOINT}

Kp=${KP}
Ki=${KI}
Kd=${KD}

COUNT=0
FILENAME="positionControlAccuracy_plot_"${PART}_${JOINT}_${COUNT}".txt"
echo "${Kp} ${Ki} ${Kd} ${FILENAME}" > PID_values.txt
run ${Kp} ${Ki} ${Kd} ${FILENAME}

for (( i=0; i<2; i++ ))
do 
    Kd=$(echo $Kd \* $(echo "$((1+RANDOM%(1-0))).$((RANDOM%999))") + $Kd | bc)
    for (( j=0; j<2; j++ ))
    do  
       Ki=$(echo $Ki \* $(echo "$((1+RANDOM%(1-0))).$((RANDOM%999))") + $Ki | bc)
       for (( k=0; k<2; k++ ))
       do
           Kp=$(echo $Kp \* $(echo "$((0+RANDOM%(1-0))).$((RANDOM%999))") + $Kp | bc)
           COUNT=`expr ${COUNT} + 1`
           FILENAME="positionControlAccuracy_plot_"${PART}_${JOINT}_${COUNT}".txt"
           echo "${Kp} ${Ki} ${Kd} ${FILENAME}" >> PID_values.txt
           run ${Kp} ${Ki} ${Kd} ${FILENAME}
       done
       Kp=${KP}
    done
    Ki=${KI}
done
}

all_joints() {
JOINTS_SIZE=22
JOINT_LIST=(head neck torso_1 torso_2 torso_3 torso_4 left_shoulder_internal left_shoulder left_elbow left_wrist right_shoulder_internal right_shoulder right_elbow right_wrist left_hip left_knee left_ankle left_ball_foot right_hip right_knee right_ankle right_ball_foot)
for (( n=0; n<"${JOINTS_SIZE}"; n++ ))
do
    tune ${JOINT_LIST[n]}
done
}

run() {

#echo $1 $2 $3 $4 $5
testrunner --test PositionControlAccuracy.so -p "--robot ${ROBOT} --part ${PART} --joints ""(${JOINT})"" --zeros ""(0)"" --step 5 --cycles 5 --sampleTime 0.01 --Kp $1 --Ki $2 --Kd $3 --filename $4"

}

usage() {
cat << EOF
***************************************************************************************
CONTROLBOARD PID TUNING SCRIPT
Author:  Yeshasvi Tirupachuri   <yeshasvi.tirupachuri@iit.it> 
This script does the control board PID tuning of a robot part using positionControl-accuracy test of icub-tests
USAGE:
        $0 options
***************************************************************************************
OPTIONS: <tuning specifier> robot, part, joint, Kp, Ki, Kd
***************************************************************************************
EXAMPLE USAGE: ./pidTuning single icubSim head 1 2 0.2 0.1 -----> PID tuning of a single joint
EXAMPLE YSAGE: ./pitTuning all icubSim 2 0.2 0.1-----> PID tuning of all the joints
***************************************************************************************
EOF
}

################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
if [[ $# -eq 0 || $# -gt 7 ]] ; then
    echo "Error in options passed!"
    echo ""
    usage
    exit 1
fi

DATA_DIR="tuning_data"

cd ..
if [[ ! -d "${DATA_DIR}" ]] ; then
    echo "Making ${DATA_DIR} directory"
    mkdir ${DATA_DIR}
fi

cd ${DATA_DIR}

if [[ "$1" == "single" ]] ; then

    echo "Tuning a single joint"
    ROBOT=$2
    PART=$3
    JOINT=$4

    KP=$5
    KI=$6
    KD=$7

    tune

elif [[ "$1" == "all" ]] ; then

    echo "Tuning all the joints"
    ROBOT=$2
    KP=$3
    KI=$4
    KD=$5

    all_joints
    
fi

exit 0
