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

####param yaml file##############
function parse_yaml {
   local prefix=$2
   local separator=${3:-_}
   local indexfix
   # Detect awk flavor
   if awk --version 2>&1 | grep -q "GNU Awk" ; then
      # GNU Awk detected
      indexfix=-1
   elif awk -Wv 2>&1 | grep -q "mawk" ; then
      # mawk detected
      indexfix=0
   fi

   local s='[[:space:]]*' sm='[ \t]*' w='[a-zA-Z0-9_]*' fs=${fs:-$(echo @|tr @ '\034')} i=${i:-  }
   cat $1 | \
   awk -F$fs "{multi=0;
       if(match(\$0,/$sm\|$sm$/)){multi=1; sub(/$sm\|$sm$/,\"\");}
       if(match(\$0,/$sm>$sm$/)){multi=2; sub(/$sm>$sm$/,\"\");}
       while(multi>0){
           str=\$0; gsub(/^$sm/,\"\", str);
           indent=index(\$0,str);
           indentstr=substr(\$0, 0, indent+$indexfix) \"$i\";
           obuf=\$0;
           getline;
           while(index(\$0,indentstr)){
               obuf=obuf substr(\$0, length(indentstr)+1);
               if (multi==1){obuf=obuf \"\\\\n\";}
               if (multi==2){
                   if(match(\$0,/^$sm$/))
                       obuf=obuf \"\\\\n\";
                       else obuf=obuf \" \";
               }
               getline;
           }
           sub(/$sm$/,\"\",obuf);
           print obuf;
           multi=0;
           if(match(\$0,/$sm\|$sm$/)){multi=1; sub(/$sm\|$sm$/,\"\");}
           if(match(\$0,/$sm>$sm$/)){multi=2; sub(/$sm>$sm$/,\"\");}
       }
   print}" | \
   sed  -e "s|^\($s\)?|\1-|" \
       -ne "s|^$s#.*||;s|$s#[^\"']*$||;s|^\([^\"'#]*\)#.*|\1|;t1;t;:1;s|^$s\$||;t2;p;:2;d" | \
   sed -ne "s|,$s\]$s\$|]|" \
        -e ":1;s|^\($s\)\($w\)$s:$s\(&$w\)\?$s\[$s\(.*\)$s,$s\(.*\)$s\]|\1\2: \3[\4]\n\1$i- \5|;t1" \
        -e "s|^\($s\)\($w\)$s:$s\(&$w\)\?$s\[$s\(.*\)$s\]|\1\2: \3\n\1$i- \4|;" \
        -e ":2;s|^\($s\)-$s\[$s\(.*\)$s,$s\(.*\)$s\]|\1- [\2]\n\1$i- \3|;t2" \
        -e "s|^\($s\)-$s\[$s\(.*\)$s\]|\1-\n\1$i- \2|;p" | \
   sed -ne "s|,$s}$s\$|}|" \
        -e ":1;s|^\($s\)-$s{$s\(.*\)$s,$s\($w\)$s:$s\(.*\)$s}|\1- {\2}\n\1$i\3: \4|;t1" \
        -e "s|^\($s\)-$s{$s\(.*\)$s}|\1-\n\1$i\2|;" \
        -e ":2;s|^\($s\)\($w\)$s:$s\(&$w\)\?$s{$s\(.*\)$s,$s\($w\)$s:$s\(.*\)$s}|\1\2: \3 {\4}\n\1$i\5: \6|;t2" \
        -e "s|^\($s\)\($w\)$s:$s\(&$w\)\?$s{$s\(.*\)$s}|\1\2: \3\n\1$i\4|;p" | \
   sed  -e "s|^\($s\)\($w\)$s:$s\(&$w\)\(.*\)|\1\2:\4\n\3|" \
        -e "s|^\($s\)-$s\(&$w\)\(.*\)|\1- \3\n\2|" | \
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\(---\)\($s\)||" \
        -e "s|^\($s\)\(\.\.\.\)\($s\)||" \
        -e "s|^\($s\)-$s[\"']\(.*\)[\"']$s\$|\1$fs$fs\2|p;t" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p;t" \
        -e "s|^\($s\)-$s\(.*\)$s\$|\1$fs$fs\2|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\?\(.*\)$s\$|\1$fs\2$fs\3|" \
        -e "s|^\($s\)[\"']\?\([^&][^$fs]\+\)[\"']$s\$|\1$fs$fs$fs\2|" \
        -e "s|^\($s\)[\"']\?\([^&][^$fs]\+\)$s\$|\1$fs$fs$fs\2|" \
        -e "s|$s\$||p" | \
   awk -F$fs "{
      gsub(/\t/,\"        \",\$1);
      if(NF>3){if(value!=\"\"){value = value \" \";}value = value  \$4;}
      else {
        if(match(\$1,/^&/)){anchor[substr(\$1,2)]=full_vn;getline};
        indent = length(\$1)/length(\"$i\");
        vname[indent] = \$2;
        value= \$3;
        for (i in vname) {if (i > indent) {delete vname[i]; idx[i]=0}}
        if(length(\$2)== 0){  vname[indent]= ++idx[indent] };
        vn=\"\"; for (i=0; i<indent; i++) { vn=(vn)(vname[i])(\"$separator\")}
        vn=\"$prefix\" vn;
        full_vn=vn vname[indent];
        if(vn==\"$prefix\")vn=\"$prefix$separator\";
        if(vn==\"_\")vn=\"__\";
      }
      assignment[full_vn]=value;
      if(!match(assignment[vn], full_vn))assignment[vn]=assignment[vn] \" \" full_vn;
      if(match(value,/^\*/)){
         ref=anchor[substr(value,2)];
         if(length(ref)==0){
           printf(\"%s=\\\"%s\\\"\n\", full_vn, value);
         } else {
           for(val in assignment){
              if((length(ref)>0)&&index(val, ref)==1){
                 tmpval=assignment[val];
                 sub(ref,full_vn,val);
                 if(match(val,\"$separator\$\")){
                    gsub(ref,full_vn,tmpval);
                 } else if (length(tmpval) > 0) {
                    printf(\"%s=\\\"%s\\\"\n\", val, tmpval);
                 }
                 assignment[val]=tmpval;
              }
           }
         }
      } else if (length(value) > 0) {
         printf(\"%s=\\\"%s\\\"\n\", full_vn, value);
      }
   }END{
      for(val in assignment){
         if(match(val,\"$separator\$\"))
            printf(\"%s=\\\"%s\\\"\n\", val, assignment[val]);
      }
   }"
}

param_path=/home/ginger/ginlt_param/
yaml_path=/vendor/ginger_robot/install/share/cmnetfw_client/config/
echo "param_path: ${param_path}"
echo "yaml_path: ${yaml_path}"

if [ -d ${yaml_path} ]; then
    eval $(parse_yaml $yaml_path/robot_config.yaml)
    config_speedHigh=$moveSpeed_gearSpeed_high
    config_speedMiddle=$moveSpeed_gearSpeed_middle
    config_speedLow=$moveSpeed_gearSpeed_low
    config_turnHigh=$moveSpeed_turnSpeed_high
    config_turnMiddle=$moveSpeed_turnSpeed_middle
    config_turnLow=$moveSpeed_turnSpeed_low
else
    log_info "${yaml_path} does not exists"
fi

if [ -d ${param_path} ]; then
    eval $(parse_yaml $param_path/robot_config.yaml)
    param_speedHigh=$moveSpeed_gearSpeed_high
    param_speedMiddle=$moveSpeed_gearSpeed_middle
    param_speedLow=$moveSpeed_gearSpeed_low
    param_turnHigh=$moveSpeed_turnSpeed_high
    param_turnMiddle=$moveSpeed_turnSpeed_middle
    param_TurnLow=$moveSpeed_turnSpeed_low
fi

if [ -d ${param_path} ]; then
    echo "${param_path} directory exists"
    if [ $(echo "($config_speedHigh - $param_speedHigh)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_speedHigh - $param_speedHigh)<=-0.001"|bc) -eq 1 ] \
      || [ $(echo "($config_speedMiddle - $param_speedMiddle)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_speedMiddle - $param_speedMiddle)<=-0.001"|bc) -eq 1 ] \
      || [ $(echo "($config_speedLow - $param_speedLow)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_speedLow - $param_speedLow)<=-0.001"|bc) -eq 1 ] \
      || [ $(echo "($config_turnHigh - $param_turnHigh)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_turnHigh - $param_turnHigh)<=-0.001"|bc) -eq 1 ] \
      || [ $(echo "($config_turnMiddle - $param_turnMiddle)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_turnMiddle - $param_turnMiddle)<=-0.001"|bc) -eq 1 ] \
      || [ $(echo "($config_turnLow - $param_TurnLow)>=0.001"|bc) -eq 1 ] || [ $(echo "($config_turnLow - $param_TurnLow)<=-0.001"|bc) -eq 1 ] ;then
        log_info "${param_path} change, so cp *.yaml files"
        cp ${yaml_path}/*.yaml ${param_path}
    else
        log_info "${param_path} no change"
    fi
else
    log_info "${param_path} does not exists, so let's  mkdir it and cp *.yaml files"
    mkdir -p ${param_path}
    cp ${yaml_path}/*.yaml ${param_path}
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

CAN_Up() {
  m=0
  echo "up can0"
  while /bin/true
  do
    if [[ $(ifconfig | grep "can0") ]]; then
      echo "can0 up success"; break
    else
      let m=m+1
      if [[ $m -gt 5 ]]; then
        echo "can0 up failed, try $m"; break
      fi
      echo 'ginger'|sudo -S modprobe can
      echo 'ginger'|sudo -S modprobe can-raw
      echo 'ginger'|sudo -S modprobe mttcan
      echo 'ginger'|sudo -S ip link set can0 up type can bitrate 1000000
      echo "can0 up waiting ..."
    fi
    sleep 2
  done
  sleep 3
}

APP_CCU_Start() {
    navigation_version=1
    if [ -f "/vendor/docker_version.prop" ];then
        log_info "navigation_version 2"
        navigation_version=2
    else
        log_info "navigation_version 1"
        navigation_version=1
    fi


    log_info "start cmnetfw_client"
    ${UN_BUF} roslaunch --wait cmnetfw_client chassis.launch > ${log_path}/${current_day}/${current_date}/cmnetfw_client.log 2>&1 &
    sleep 2

    log_info "start ginger_charge"
    ${UN_BUF} roslaunch --wait ginger_charge auto_dock.launch > ${log_path}/${current_day}/${current_date}/glite_charge.log 2>&1  &

    log_info "start ginger_chassis_lite"
    ${UN_BUF} roslaunch --wait ginger_chassis ginger_chassis.launch > ${log_path}/${current_day}/${current_date}/glite_chassis.log 2>&1 &
    sleep 2

    log_info "start glite_navigation_sensors"
    if [[ $hw105 -eq 1 ]]; then
        if [[ $navigation_version -eq 1 ]]; then
            log_info "start navi_sensors_hw105"
            ${UN_BUF} roslaunch --wait glite_navigation  navi_sensors_hw105.launch > ${log_path}/${current_day}/${current_date}/glite_sensors.log 2>&1 &
        else
            log_info "start navi2_sensors_hw105"
            ${UN_BUF} roslaunch --wait glite_navigation  navi2_sensors_hw105.launch > ${log_path}/${current_day}/${current_date}/glite_sensors.log 2>&1 &
        fi
    else
        if [[ $navigation_version -eq 1 ]]; then
            log_info "start glite_navi_sensors"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_navi_sensors.launch > ${log_path}/${current_day}/${current_date}/glite_sensors.log 2>&1 &
        else
            log_info "start glite_navi2_sensors"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_navi2_sensors.launch > ${log_path}/${current_day}/${current_date}/glite_sensors.log 2>&1 &
        fi
    fi
    sleep 1

    if [[ $hw105 -eq 1 ]]; then
        log_info "start mx_camera"
        ${UN_BUF} roslaunch --wait mx_camera raw_server.launch > ${log_path}/${current_day}/${current_date}/glite_mx_camera.log 2>&1 &

        log_info "start suspend_obstacle_detection"
        ${UN_BUF} roslaunch --wait suspend_obstacle_detection suspend_scan.launch > ${log_path}/${current_day}/${current_date}/glite_suspend_obstacle_detection.log 2>&1 &

        log_info "start fisheye_camera"
        ${UN_BUF} roslaunch --wait uvc_camera fisheye_camera_node.launch > ${log_path}/${current_day}/${current_date}/glite_fisheye_camera.log 2>&1 &
    fi

    log_info "start lpms_imu"
    ${UN_BUF} roslaunch --wait lpms_ig1 lpms_be1.launch > ${log_path}/${current_day}/${current_date}/glite_lpmsimu.log 2>&1  &
    sleep 1

    log_info "start robot_pose_ekf"
    ${UN_BUF} roslaunch robot_pose_ekf robot_pose_ekf.launch > ${log_path}/${current_day}/${current_date}/glite_robot_pose_ekf.log 2>&1  &
    sleep 1

    # log_info "start imu and wheel odom fusion"
    # ${UN_BUF} roslaunch odom_imu_fusion odom_imu_fusion.launch > ${log_path}/${current_day}/${current_date}/glite_imu_odom_fusion.log 2>&1  &
    # sleep 1

    # log_info "start ginger_vslam"
    # ${UN_BUF} roslaunch ginger_vslam  ginger_vslam.launch > ${log_path}/${current_day}/${current_date}/glite_vslam.log 2>&1 &
    # sleep 1

    # log_info "start ginger_virtualwall"
    # ${UN_BUF} roslaunch --wait virtual_wall virtual.launch > ${log_path}/${current_day}/${current_date}/glite_virtualwall.log 2>&1 &
    # sleep 1

#    log_info "start move_base"
#    if [[ $hw105 -eq 1 ]]; then
#        ${UN_BUF} roslaunch --wait move_base  move_base_hw105.launch > ${log_path}/${current_day}/${current_date}/glite_movebase.log 2>&1 &
#    elif [[ $pvt3p2 -eq 1 ]]; then
#        ${UN_BUF} roslaunch --wait move_base  move_base_box.launch > ${log_path}/${current_day}/${current_date}/glite_movebase.log 2>&1 &
#    else
#        ${UN_BUF} roslaunch --wait move_base  move_base.launch > ${log_path}/${current_day}/${current_date}/glite_movebase.log 2>&1 &
#    fi
#    sleep 1

    # log_info "start ginger_lidar_slam"
    # ${UN_BUF} roslaunch --wait lidar_slam lidar_slam.launch > ${log_path}/${current_day}/${current_date}/glite_lidarslam.log 2>&1 &
    # sleep 1

    log_info "start glite_record"
    ${UN_BUF} roslaunch --wait glite_navigation  ginger_record.launch > ${log_path}/${current_day}/${current_date}/glite_recordbag.log 2>&1 &
    sleep 1

    log_info "start glite_navigation"
    if [[ $hw105 -eq 1 ]]; then
        if [[ $navigation_version -eq 1 ]]; then
            log_info "start glite_navigation_hw110"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_navigation_hw110.launch > ${log_path}/${current_day}/${current_date}/glite_navi.log 2>&1 &
            sleep 1

            log_info "start lidar_slam_node"
            ${UN_BUF} roslaunch --wait lidar_slam lidar_slam.launch > ${log_path}/${current_day}/${current_date}/glite_lidarslam.log 2>&1 &
            sleep 1
        else
            log_info "start glite_navigation2_hw110"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_navigation2_hw110.launch > ${log_path}/${current_day}/${current_date}/glite_navi.log 2>&1 &
        fi
    else
        if [[ $navigation_version -eq 1 ]]; then
            log_info "start glite_navigation"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_navigation.launch > ${log_path}/${current_day}/${current_date}/glite_navi.log 2>&1 &
            sleep 1

            log_info "start lidar_slam_node"
            ${UN_BUF} roslaunch --wait lidar_slam lidar_slam.launch > ${log_path}/${current_day}/${current_date}/glite_lidarslam.log 2>&1 &
            sleep 1
        else
            log_info "start glite_sps"
            ${UN_BUF} roslaunch --wait glite_navigation  glite_sps.launch > ${log_path}/${current_day}/${current_date}/glite_navi.log 2>&1 &
        fi
    fi
    sleep 15

    log_info "start grpc_server"
    ${UN_BUF} roslaunch --wait cm_ginger cm_ginger.launch > ${log_path}/${current_day}/${current_date}/glite_cmgrpc.log 2>&1  &

    log_info "start ginger_alarm"
    ${UN_BUF} roslaunch --wait ginger_alarm ginger_alarm.launch > ${log_path}/${current_day}/${current_date}/glite_alarm.log 2>&1  &

    log_info "start ginger_status"
    ${UN_BUF} roslaunch --wait ginger_status ginger_status.launch > ${log_path}/${current_day}/${current_date}/glite_status.log 2>&1  &

    log_info "start ginger_performance"
    ${UN_BUF} roslaunch --wait ginger_performance ginger_performance.launch > ${log_path}/${current_day}/${current_date}/glite_performance.log 2>&1  &

    #log_info "start yolov5_ros"
    #${UN_BUF} roslaunch --wait yolov5_ros yolo_ros.launch > ${log_path}/${current_day}/${current_date}/glite_yolov5_ros.log 2>&1  &

    #${UN_BUF} roslaunch head_demo head_demo.launch > ${log_path}/${current_day}/${current_date}/head_demo.log 2>&1  &

    #rostopic pub -1 /joint_states sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['Head_Y'], position: [0.0], velocity: [], effort: []}'
    #sleep 1
    #rostopic pub -1 /joint_states sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['Head_Z'], position: [0.0], velocity: [], effort: []}'
    #sleep 1

    #node 1: cm_dds_server
    log_info "roslaunch cm_dds_v2 cm_dds_client.launch > ~/dds_client.log 2>&1"
    ${UN_BUF} roslaunch cm_dds_v2 cm_dds_client.launch > ${log_path}/${current_day}/${current_date}/dds_client.log 2>&1 &
    sleep 1

    log_info "start glite_hdmap"
    ROSCONSOLE_FORMAT='[${severity}][${time}][${node}]: ${message}' && ${UN_BUF} rosrun hd_map_service server.py > ${log_path}/${current_day}/${current_date}/glite_hdmap.log 2>&1 &
    sleep 1


    if [[ "$(docker inspect nav2 2> /dev/null | grep '"Name": "/nav2"')" != "" ]]; then
        log_info "docker stop nav2"
        docker stop nav2
        sleep 8

        log_info "docker start nav2"
        docker start nav2
    else
        log_info "navigation2 container not ready dont start it!"
    fi

    if [[ "$(docker inspect localization 2> /dev/null | grep '"Name": "/localization"')" != "" ]]; then
        log_info "docker stop localization"
        docker stop localization
        sleep 8

        log_info "docker start localization"
        docker start localization
    else
        log_info "localization container not ready dont start it!"
    fi


}

APP_ACU_Start() {

    CAN_Up

    #node 1: cm_dds_server
    log_info "roslaunch cm_dds_v2 cm_dds_server.launch > ~/dds_server.log 2>&1"
    ${UN_BUF} roslaunch cm_dds_v2 cm_dds_server.launch > ${log_path}/${current_day}/${current_date}/dds_server.log 2>&1 &
    sleep 1

    #node 2: dual_arm_actuator
    log_info "roslaunch dual_arm_actuator dual_arm_actuator.launch > ~/dual_arm_actuator.log 2>&1"
    ${UN_BUF} roslaunch dual_arm_actuator dual_arm_actuator.launch > ${log_path}/${current_day}/${current_date}/dual_arm_actuator.log 2>&1 &
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
        APP_ACU_Start
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
