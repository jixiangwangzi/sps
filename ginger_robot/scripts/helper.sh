#!/bin/bash

LOG_DIR=/home/ginger
LOG_FILE=${LOG_DIR}/shell_1.log

function rotate_logs ()
{
	log_index=5
	while [  ${log_index} -gt 0 ]; do
		if [ -f shell_${log_index}.log ]
		then
			log_info "rotate shell_${log_index}.log"
			let target_log_index=log_index+1
			mv ${LOG_DIR}/shell_${log_index}.log ${LOG_DIR}/shell_${target_log_index}.log
		fi
		let log_index=log_index-1
	done
}

function log_info ()
{
	DATE_N=`date "+%Y-%m-%d %H:%M:%S"`
	USER_N=`whoami`
	LOG_MSG="${DATE_N} ${USER_N} [INFO] $@"
	echo "${LOG_MSG}"
	echo "${LOG_MSG}" >> $LOG_FILE
}

function log_error ()
{
	DATE_N=`date "+%Y-%m-%d %H:%M:%S"`
	USER_N=`whoami`
	LOG_MSG="${DATE_N} ${USER_N} [ERROR] $@"
	echo -e "\033[41;37m ${LOG_MSG} \033[0m"
	echo -e "${LOG_MSG}"  >> $LOG_FILE
}

function fn_log ()
{
	if [  $? -eq 0  ]
	then
	    log_info "$@ sucessed."
	else
	    log_error "$@ failed."
	    exit 1
	fi
}
