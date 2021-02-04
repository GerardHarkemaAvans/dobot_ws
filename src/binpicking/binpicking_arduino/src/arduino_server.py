#!/usr/bin/env python
# license removed for brevity
import rospy
import rosservice

from std_msgs.msg import Bool, Empty ,String
from binpicking_arduino.srv import isButtonPressed ,isButtonPressedRequest ,isButtonPressedResponse
from binpicking_arduino.srv import setBlinking, setBlinkingRequest ,setBlinkingResponse
from binpicking_arduino.srv import isCalibrating, isCalibratingRequest ,isCalibratingResponse

lamp = False
isReady = False
blink = False
isCalibratePressed = False
prevHardwareReady = False
isStartingIdle = False
isInIdle = False

def callback(data):
    global lamp
    global isReady
    global isInIdle
    global prevHardwareReady


    if not isInIdle:
        prevHardwareReady = True

    #if everything is ready and the data is true then set the light to true
    if data.data and isReady and prevHardwareReady:
        lamp = not lamp
        #pub.publish()
#process the service for the check if the button has been pressed 
#and also check if the hardware is ready 
def process_service_request(req):
    global lamp
    global prevHardwareReady
    global blink
    global isStartingIdle

    res = isButtonPressedResponse()
    res.isPressed = lamp

    toCheck= ["/camera1/depth/color/points","/camera2/depth/color/points","magician/moveToPosition" ] #,"arduino_toggle_led"]
    topics = rospy.get_published_topics()
    topic_string = str(topics)
    service_list = rosservice.get_service_list()
    service_string = str(service_list)
    topics_and_services = topic_string + service_string

    hardwareReady = all(x in topics_and_services for x in toCheck)
    #rospy.loginfo("HardwareReady: %s",str(hardwareReady))
    if not prevHardwareReady == hardwareReady or isStartingIdle:
        prevHardwareReady = hardwareReady
        isStartingIdle = False
        if not hardwareReady:
            rospy.loginfo(res)
            if blink:
                blink = False
                #pubBlink.publish()
        if hardwareReady and not blink and isReady:
            blink = True
            #pubBlink.publish()
    

    return res

#Process the request for enabling blinking
def process_blink_service_request(req):
    global blink
    res = setBlinkingResponse()
    if req.blink and not blink:
        blink = True
        pubBlink.publish()
        res.succes = True
    elif not req.blink and blink:
        blink = False
        pubBlink.publish()
        res.succes = True
    else:
        res.succes = False

    return res

#Callback from the statemachine
def flexbe_behaviour_update(data):
    global isReady
    global isStartingIdle
    global isInIdle
    if data.data == "/idle":
        isReady = True
        isStartingIdle = True
        isInIdle = True
    elif data.data == "/calibrate" or data.data =="/reconnect" :
        isReady = False
        isInIdle = False
    else:
        isInIdle = False

#Callback for calibrate
def calibrate(data):
    global isCalibratePressed
    isCalibratePressed = data.data

def process_calibrate_request(req):
    global calibrate
    res = isCalibratingResponse()
    res.isCalibratingPressed = isCalibratePressed
    return res

if __name__=='__main__':
    rospy.init_node('ardruino', anonymous=True)

    rospy.Subscriber("button_pushed", Bool, callback)
    rospy.loginfo("Started button_pushed subscriber in arduino_server")
    
    rospy.Subscriber("flexbe/behavior_update", String, flexbe_behaviour_update)
    rospy.loginfo("Started flexbe/behavior_update subscriber in arduino_server")
    
    rospy.Subscriber("calibrate", Bool, calibrate)
    rospy.loginfo("Started calibrate subscriber in arduino_server")

#    pub = rospy.Publisher('arduino_toggle_led', Empty, queue_size=10)
#    rospy.loginfo("Started arduino_toggle_led publischer in arduino_server")

#    pubBlink = rospy.Publisher('arduino_blink_led', Empty, queue_size=10)
#    rospy.loginfo("Started arduino_blink_led publischer in arduino_server")

    service = rospy.Service('binpicking/button', isButtonPressed, process_service_request)
    rospy.loginfo("Started binpicking/button service")

    service = rospy.Service('binpicking/blink', setBlinking, process_blink_service_request)
    rospy.loginfo("Started binpicking/blink service")

    service = rospy.Service('binpicking/calibrate', isCalibrating, process_calibrate_request)
    rospy.loginfo("Started binpicking/calibrate service")



    rospy.spin()
