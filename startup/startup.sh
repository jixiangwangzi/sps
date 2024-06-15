#!/bin/bash
set -e

ln -snf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
UN_BUF="stdbuf -oL -eL "
current_day=`date +%m%d`
current_date=`date +%H.%M.%S`
log_path=/home/ginger/docker_logs/
LOG_RETENTION_DAYS=15
STARTUP_LOG_FILE=${log_path}/${current_day}/${current_date}/startup.log


function log_info ()
{
    DATE_N=`date "+%Y-%m-%d %H:%M:%S"`
    USER_N=`whoami`
    LOG_MSG="[ INFO] [ ${DATE_N}] ${USER_N}: $@"
    echo "${LOG_MSG}" >> $STARTUP_LOG_FILE
}


mkdir -p ${log_path}/${current_day}/${current_date}/
ln -snf ${log_path}/${current_day}/${current_date} /root/latest

log_info "startup start ..."

DATE_TMP=$(date -d "${current_day} ${LOG_RETENTION_DAYS} days ago" "+%m%d")
log_info "logs expiration date is `date "+%Y"`${DATE_TMP}"
ls ${log_path}|
while read line
do
    log_info "deal with logs dir: ${log_path}/${line}"
    if [ $(echo $line | awk '{print($0~/^[-]?([0-9])+[.]?([0-9])+$/)?$line:9999}') -gt ${DATE_TMP} ]; then
        log_info "recently ${LOG_RETENTION_DAYS} days log file, should retain "
    else
        log_info "old logs beyond ${LOG_RETENTION_DAYS} days should removed, now exec 'rm -rf ${log_path}/${line}/'"
        rm -rf ${log_path}/${line}/
    fi
done

source /vendor/$ROS1_DISTRO/setup.bash
source /vendor/$ROS2_DISTRO/setup.bash

eth0_ip=$(/sbin/ifconfig eth0|grep inet|grep -v inet6|awk '{print $2}'|tr -d "addr:")
log_info "eth0_ip: "${eth0_ip}
if [ "$eth0_ip" = "192.168.137.200" ]; then
  export ROS_MASTER_URI=http://192.168.137.200:11311
  export ROS_IP=192.168.137.200
elif [ "$eth0_ip" = "192.168.1.200" ]; then
  export ROS_MASTER_URI=http://192.168.1.200:11311
  export ROS_IP=192.168.1.200
elif [ "$eth0_ip" = "192.168.1.20" ]; then
  export ROS_MASTER_URI=http://192.168.1.20:11311
  export ROS_IP=192.168.1.20
else
  export ROS_MASTER_URI=http://192.168.137.200:11311
  export ROS_IP=${eth0_ip}
fi

if [ "$ROBOT_MODEL" = "patrol" ]; then
  export ROS_MASTER_URI=http://192.168.1.20:11311
fi

log_info "ROS_MASTER_URI: "$ROS_MASTER_URI
log_info "ROS_HOSTNAME: "$ROS_HOSTNAME
log_info "ROS_IP: "$ROS_IP
export ROSCONSOLE_FORMAT='[${severity}] [${time: %Y-%m-%d %H:%M:%S.%f %z}]: ${message}'
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

log_info ""
log_info "============================================"

log_info "start parameter_bridge"
rosparam load /vendor/$ROS2_DISTRO/share/cm_msgs/parameter_bridge.yaml
${UN_BUF} ros2 run ros1_bridge parameter_bridge --ros-args --log-level info > ${log_path}/${current_day}/${current_date}/parameter_bridge.log 2>&1 &
sleep 1


# log_info "start depthscan_to_pointcloud2"
# ${UN_BUF} ros2 launch -n -a pointcloud_to_laserscan depthscan_to_pointcloud2.py > ${log_path}/${current_day}/${current_date}/depthscan_to_pointcloud2.log 2>&1 &
# sleep 1

log_info "start navigation2"
${UN_BUF} ros2 launch -n -a  nav2_launch sps_nav_launch.py log_level:=INFO > ${log_path}/${current_day}/${current_date}/navigation_launch.log 2>&1 &
sleep 1

log_info "start sps"
${UN_BUF} ros2 launch -n -a sps_launch sps_launch.py log_level:=INFO > ${log_path}/${current_day}/${current_date}/sps_launch.log 2>&1 &
sleep 1

log_info "============================================"
log_info ""

log_info "startup complete"
tail -f $STARTUP_LOG_FILE
exec "$@"
