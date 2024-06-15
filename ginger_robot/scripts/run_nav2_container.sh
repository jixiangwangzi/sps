#!/bin/bash

registry_name="index.cloudminds.com"
image_name="ccu/nav2"

if [ ! -z "$1" ]; then
	nav2_version=$1
else
	nav2_version=`grep -r "nav2_version" /vendor/buildinfo.prop | awk -F '=' '{print $2}'`
fi
	
echo "$nav2_version"
sudo docker login -u ccu -p F4kzIdOp $registry_name
echo "docker pull $registry_name/$image_name:${nav2_version}"
if [ `sudo docker ps -a | grep "nav2" | awk '{print $1}'` ]; then
	echo "delete current container"
        sudo docker ps -a | grep "nav2" | awk '{print $1}' | xargs sudo docker rm -f
fi
sudo docker run -it --net host --volume /home/ginger:/home/ginger --name nav2 -d $registry_name/$image_name:$nav2_version /bin/bash
echo "start new container"
