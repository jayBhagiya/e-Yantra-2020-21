from multiprocessing.dummy import Pool
import time
import requests

import sys
import paho.mqtt.client as mqtt #import the client1
import time

class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    try:        
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1


# -----------------  SHEET PUSH -------------------
def push_data(*argv):
        URL = "https://script.google.com/macros/s/" + str(argv[0]) + "/exec"
        parameters = {"id":"Sheet1", "turtle_x":argv[1], "turtle_y":argv[2], "turtle_theta":argv[3]}

        # url1 = "https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"
        # parameters1 = {"id":"task1", "team_id":"VB_0691", "unique_id":"jshBCD", "turtle_x":argv[1], "turtle_y":argv[2], "turtle_theta":argv[3]} 

        responce = requests.get(URL, params=parameters)
        # responce1 = requests.get(url1, params=parameters1)

        print(responce.content)