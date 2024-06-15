#!/bin/bash
project_path=/vendor
echo "progect_path: ${project_path}"
source ${project_path}/ginger_robot/scripts/helper.sh

if [[ -z "${ROBOT_DIR}" ]]; then
    echo "ROBOT_DIR not defined"
    exit -1
fi

####for hw compatible####
hw105=0
pvt3p2=0

####network sync##############
log_info "network sync..."
timeout=0
result=""
if [[ $(cat /sbin/subsystem_init.bash | grep ginger.service | grep sync_time) ]]; then
    log_info "network sync has been finished before, don't sync again"
else
    while true
    do
        result=$(timedatectl | grep "System clock synchronized" | awk -F ': ' '{print $2}')
        log_info "System clock synchronized: $result"
        if [[ $result == "yes" ]]; then
            log_info "network sync successed!"
	        break
        else
            let timeout=timeout+1
            if [[ $timeout -ge 100 ]]; then
                log_info "network sync failed!"
                break
            fi
            sleep 1
        fi
    done
fi

####set for tx2 logs##############
log_path=/home/ginger/ginger_logs/
echo "log_path: ${log_path}"

if [ -d ${log_Path} ]; then
    echo "${log_path} directory exists"
else
    echo "${log_path} does not exists, so let's  mkdir it"
    mkdir -p ${log_path}
fi

current_day=`date +%m%d`
echo "current day: "${current_day}
current_date=`date +%H.%M.%S`
echo "current date: "${current_date}

echo "mkdir directory about day and time"

mkdir -p ${log_path}/${current_day}/${current_date}/

echo "creat soft link"
ln -snf ${log_path}/${current_day}/${current_date} /home/ginger/latest

NUMBER_OFFSET=10 #delete ten days ago log file
echo "days offset to save log files: "${NUMBER_OFFSET}

ls ${log_path}|
while read line
do
    echo "line is " + ${line}
    tmp=`expr ${line} + ${NUMBER_OFFSET}`
    echo "temp is " + ${tmp}
    if [ ${tmp} -gt ${current_day} ]; then
        echo "${tmp} > > > ${current_day}"
    	echo "recently two day's log file, should retain "
    else
        echo "${tmp} <<<< ${current_day}"
    	echo "warn: [ old logs, should remove ]"
	    rm -rf ${log_path}/${line}/
    fi
done

##################end for logs setting##############
echo "$@: " $@

cd_to () {
  cd $@
}

######  copied from bashrc ###########
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nvidia/mwc_ws/apriltag/build/lib
distribution=$(lsb_release -c -s)
log_info "distribution:"${distribution}
if [[ $distribution == "focal" ]]; then
    source /opt/ros/noetic/setup.bash
else
    source /opt/ros/melodic/setup.bash
fi
eth0_ip=$(/sbin/ifconfig eth0|grep inet|grep -v inet6|awk '{print $2}'|tr -d "addr:")
log_info "eth0_ip:"${eth0_ip}
export ROS_MASTER_URI=http://${eth0_ip}:11311
export ROS_IP=${eth0_ip}
export ROSCONSOLE_FORMAT='[${severity}] [${time: %Y-%m-%d %H:%M:%S.%f %z}]: ${message}'
#################

${project_path}/ginger_robot/scripts/helper.sh

rotate_logs

log_info "rosclean purge -y"
rosclean purge -y

log_info "Enter start_ginger_lite_service 1"

echo "ROBOT_DIR:"${ROBOT_DIR}
cd_to ${ROBOT_DIR}
. install/setup.bash

sleep 3

UN_BUF="stdbuf -oL -eL "
log_info "roscore > ~/roscore.log 2>&1"
${UN_BUF} roscore > ${log_path}/${current_day}/${current_date}/roscore.log 2>&1 &
count=1
while /bin/true
do
  if [ $(rosnode list 2> /dev/null | grep "/rosout") ]; then
    log_info "roscore: success"; break
  else
    log_info "roscore: wait..."
  fi
  sleep 1 # wait for 1 second

  if [ ${count} -eq 20 ]; then
    log_info "retry count: ${count}"; break
  else
    count=$((${count} + 1))
  fi
done

set -e # exit immediately if a command return a non-zero status.

APP_CCU_Start() {

log_info "start cmnetfw_client"
${UN_BUF} roslaunch cmnetfw_client chassis.launch > ${log_path}/${current_day}/${current_date}/cmnetfw_client_factory.log 2>&1 &
sleep 1

log_info "start ginger_charge"
${UN_BUF} roslaunch ginger_charge auto_dock.launch > ${log_path}/${current_day}/${current_date}/glite_charge_factory.log 2>&1  &

log_info "start ginger_chassis_lite"
${UN_BUF} roslaunch  ginger_chassis ginger_chassis.launch > ${log_path}/${current_day}/${current_date}/glite_chassis_factory.log 2>&1 &
sleep 1

log_info "start glite_navigation_sensors"
if [[ $hw105 -eq 1 ]]; then
    ${UN_BUF} roslaunch glite_navigation  navi_sensors_hw105.launch > ${log_path}/${current_day}/${current_date}/glite_sensors_factory.log 2>&1 &
else
    ${UN_BUF} roslaunch glite_navigation  glite_navi_sensors.launch > ${log_path}/${current_day}/${current_date}/glite_sensors_factory.log 2>&1 &
fi
sleep 1

if [[ $hw105 -eq 1 ]]; then
    log_info "start mx_camera"
    ${UN_BUF} roslaunch mx_camera raw_server.launch > ${log_path}/${current_day}/${current_date}/glite_mx_camera_factory.log 2>&1 &

    log_info "start suspend_obstacle_detection"
    ${UN_BUF} roslaunch suspend_obstacle_detection suspend_scan.launch > ${log_path}/${current_day}/${current_date}/glite_suspend_obstacle_detection_factory.log 2>&1 &

    log_info "start fisheye_camera"
    ${UN_BUF} roslaunch uvc_camera fisheye_camera_node.launch > ${log_path}/${current_day}/${current_date}/glite_fisheye_camera_factory.log 2>&1 &
fi

log_info "start lpms_imu"
${UN_BUF} roslaunch lpms_ig1 lpms_be1.launch > ${log_path}/${current_day}/${current_date}/glite_lpmsimu_factory.log 2>&1  &
sleep 1

log_info "start robot_pose_ekf"
${UN_BUF} roslaunch robot_pose_ekf robot_pose_ekf.launch > ${log_path}/${current_day}/${current_date}/glite_robot_pose_ekf.log 2>&1  &
sleep 1

# log_info "start ginger_vslam"
# ${UN_BUF} roslaunch ginger_vslam  ginger_vslam.launch > ${log_path}/${current_day}/${current_date}/glite_vslam_factory.log 2>&1 &
# sleep 1

log_info "start ginger_virtualwall"
${UN_BUF} roslaunch virtual_wall virtual.launch > ${log_path}/${current_day}/${current_date}/glite_virtualwall_factory.log 2>&1 &
sleep 1

log_info "start move_base"
if [[ $hw105 -eq 1 ]]; then
    ${UN_BUF} roslaunch move_base  move_base_hw105.launch > ${log_path}/${current_day}/${current_date}/glite_movebase_factory.log 2>&1 &
elif [[ $pvt3p2 -eq 1 ]]; then
    ${UN_BUF} roslaunch move_base  move_base_box.launch > ${log_path}/${current_day}/${current_date}/glite_movebase_factory.log 2>&1 &
else
    ${UN_BUF} roslaunch move_base  move_base.launch > ${log_path}/${current_day}/${current_date}/glite_movebase_factory.log 2>&1 &
fi
sleep 1

log_info "start ginger_lidar_slam"
${UN_BUF} roslaunch --wait lidar_slam lidar_slam.launch > ${log_path}/${current_day}/${current_date}/glite_lidarslam.log 2>&1 &
sleep 1

log_info "start glite_record"
${UN_BUF} roslaunch --wait glite_navigation  ginger_record.launch > ${log_path}/${current_day}/${current_date}/glite_recordbag.log 2>&1 &
sleep 1

log_info "start glite_navigation"
if [[ $hw105 -eq 1 ]]; then
    ${UN_BUF} roslaunch glite_navigation  glite_navigation_hw110.launch > ${log_path}/${current_day}/${current_date}/glite_navi_factory.log 2>&1 &
else
    ${UN_BUF} roslaunch glite_navigation  glite_navigation.launch > ${log_path}/${current_day}/${current_date}/glite_navi_factory.log 2>&1 &
fi
sleep 15

log_info "start grpc_server"
${UN_BUF} roslaunch cm_ginger cm_ginger.launch > ${log_path}/${current_day}/${current_date}/glite_cmgrpc_factory.log 2>&1  &

log_info "start ginger_alarm"
${UN_BUF} roslaunch ginger_alarm ginger_alarm.launch > ${log_path}/${current_day}/${current_date}/glite_alarm_factory.log 2>&1  &

log_info "start ginger_status"
${UN_BUF} roslaunch ginger_status ginger_status.launch > ${log_path}/${current_day}/${current_date}/glite_status_factory.log 2>&1  &

log_info "start ginger_performance"
${UN_BUF} roslaunch ginger_performance ginger_performance.launch > ${log_path}/${current_day}/${current_date}/glite_performance_factory.log 2>&1  &

log_info "start glite_patrol"
${UN_BUF} roslaunch glite_navigation ginger_patrol.launch > ${log_path}/${current_day}/${current_date}/glite_patrol_factory.log 2>&1  &

#log_info "start yolov5_ros"
#${UN_BUF} roslaunch --wait yolov5_ros yolo_ros.launch > ${log_path}/${current_day}/${current_date}/glite_yolov5_ros.log 2>&1  &

#${UN_BUF} roslaunch head_demo head_demo.launch > ${log_path}/${current_day}/${current_date}/head_demo.log 2>&1  &

#rostopic pub -1 /joint_states sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['Head_Y'], position: [0.0], velocity: [], effort: []}'
#sleep 1
#rostopic pub -1 /joint_states sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['Head_Z'], position: [0.0], velocity: [], effort: []}'
#sleep 1
}

# query board is CCU or ACU
deviceCfg_path=/ftm/device.cfg

if [ -e "$deviceCfg_path" ]; then
    board=$(cat /ftm/device.cfg | grep -e "board" | awk -F ":" '{print $NF}' |sed -e 's/[[:space:]]*//'|sed 's/\"//g' |sed 's/\r//g' |sed 's/,//g')
    mark=$(cat /ftm/device.cfg | grep -e "remark" | awk -F ":" '{print $NF}' |sed -e 's/[[:space:]]*//'|sed 's/\"//g' |sed 's/\r//g')
    hw_v=$(cat /ftm/device.cfg | grep -e "hardwareVersion" | awk -F ":" '{print $NF}' |sed -e 's/[[:space:]]*//'|sed 's/\"//g' |sed 's/\r//g' |sed 's/,//g')
    if [[ -z "$mark" || -z "$hw_v" ]]; then
        log_info "device.cfg remark or hw version is null"
    elif [[ "$mark" == "PVT" && "$hw_v" == "3.2" ]]; then
        hw105=0
        pvt3p2=1
        log_info "hw version: $mark $hw_v,new pvt3p2"
    elif [[ "$mark" == "PVT" && "$hw_v" > "3.2" ]]; then
        hw105=1
        pvt3p2=0
        log_info "hw version: $mark $hw_v,new hw105"
    else
        hw105=0
        pvt3p2=0
        log_info "hw version: $mark $hw_v,old hw"
    fi

    if [[ -z "$board" ]]; then
        log_info "board is null"
    elif [[ "$board" == "ACU" ]]; then
        log_info "i am ACU"
        #APP_ACU_Start
    elif [[ "$board" == "CCU" ]]; then
        log_info "i am CCU"
        APP_CCU_Start
    fi
else
    log_info "device.cfg does not exist"
    exit 0
fi

i=0
while true
do
    let i=i+1
    log_info "i is $i"
    sleep 30
    if [[ $i -gt 10000 ]]; then
      i=0
    fi
done

log_info "Leave ginger_lite_service 1"
