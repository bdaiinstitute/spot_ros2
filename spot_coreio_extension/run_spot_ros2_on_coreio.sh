#!/bin/bash
helpFunction()
{
   echo ""
   echo "Usage: $0 -pw password -ip ipaddr"
   echo -e "\t-p Spot password for SSH"
   echo -e "\t-i Spot ip address"
   echo -e "\t-u Upload the docker image to robot"
   exit 1 # Exit script after printing help
}

upload=0
while getopts "p:i:u" opt; do
  case ${opt} in
    p )
      pass=$OPTARG
      ;;
    i )
      ip_addr=$OPTARG
      ;;
    u )
      upload=1
      ;;
    \? )
      helpFunction
      ;;
  esac
done
# Print helpFunction in case parameters are empty
if [ -z "$pass" ] || [ -z "$ip_addr" ]
then
   echo "Some or all of the parameters are empty";
   helpFunction
fi

# Logic for uploading the docker image and compose
if [ $upload -gt 0 ]
then
 sshpass -p $pw scp -P 20022 docker-compose.yml spot@$ip:/home/spot
 sshpass -p $pw scp -P 20022 spot_ros2_amd64.tgz spot@$ip:/home/spot
 # sshpass -p $pw scp -P 20022 walk_forward.py spot@$ip:/home/spot
fi

# Assumes the image has been created and saved as spot_ros2_amd64.tgz
sshpass -p $pass ssh -p 20022 spot@$ip_addr << EOF
  echo -e $pass | sudo -S chmod 777 .
  echo -e $pass | sudo -S docker kill spot-spot_ros2-1
  echo -e $pass | sudo -S docker rm spot-spot_ros2-1
  echo -e $pass | sudo -S docker rmi -f spot_ros2:arm64
  echo "Loading docker..."
  echo -e $pass | sudo -S docker image load --input spot_ros2_arm64.tgz
  echo "Running container..."
  echo -e $pass | sudo -S docker-compose up 
  echo "Finished"
EOF
