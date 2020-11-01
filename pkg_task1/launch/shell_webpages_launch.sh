#!/bin/bash

url1="http://www.hivemq.com/demos/websocket-client/"
url2="https://docs.google.com/spreadsheets/d/1QViYcx-kovThCVxMAnVhY5f9X90m5D7FCCVrjbC19DQ/edit?usp=sharing"

echo -e "\n ---- opening both url in firefox ---- \n"

echo " MQTT Publish Topic : /eyrc/jshBCD/iot_to_ros"
echo " MQTT Subscribe Topic : /eyrc/jshBCD/ros_to_iot"
echo -e " Type [Start] in publish to start the tracing of turtle.\n"

firefox -new-window $url1 $url2