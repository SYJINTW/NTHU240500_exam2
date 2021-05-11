import paho.mqtt.client as paho
import serial
import time

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)

# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client

# MQTT broker hosted on local machine
mqttc = paho.Client()

# Settings for connection
# TODO: revise host to your IP
host = "192.168.0.24"
topic = "Mbed"

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

count = 1
def on_message(mosq, obj, msg):
    global count
    if len(str(msg.payload)) >= 21 and len(str(msg.payload)) < 30: # len(Angle threshold is %d)
        count = 1
        print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
        s.write(bytes("\r", 'UTF-8'))
        time.sleep(1)
        
        s.write(bytes("/mode_sl/run 3\r", 'UTF-8'))
        time.sleep(1)
    elif len(str(msg.payload)) < 21:
        if count >= 5:
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
            count = 1
            
            s.write(bytes("\r", 'UTF-8'))
            time.sleep(1)
            
            s.write(bytes("/mode_sl/run 3\r", 'UTF-8'))
            time.sleep(1)
        else:
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
            count = count + 1

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

# Publish messages from Python

num = 0
while num != 5:
    ret = mqttc.publish(topic, "-----Message from Python!-----\n", qos=0)
    if (ret[0] != 0):
            print("Publish failed")
    mqttc.loop()
    time.sleep(1.5)
    num += 1

# Loop forever, receiving messages
mqttc.loop_forever()