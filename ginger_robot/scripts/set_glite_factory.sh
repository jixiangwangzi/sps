#!/bin/bash


echo "start"

#echo "disable ginger.service"
#sudo systemctl disable ginger.service
echo "stop ginger.service"
sudo systemctl stop ginger.service
echo "cp glite_factory.service to /etc/systemd/system"
sudo cp /vendor/ginger_robot/scripts/glite_factory.service /etc/systemd/system/
echo "reload system daemond"
sudo systemctl daemon-reload
echo "start glite_factory.service"
sudo systemctl start glite_factory.service
echo "ok"
