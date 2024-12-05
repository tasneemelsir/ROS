#!/usr/bin/env python3
# llm_subscriber.py

import rospy
from std_msgs.msg import String
import pyttsx3  # For text-to-speech

def print_response(msg):
    # Print the text response
    rospy.loginfo(f"Response: {msg.data}")
    
    # Speak the response (optional, if output mode was set to speech)
    engine = pyttsx3.init()  # Initialize the TTS engine
    engine.say(msg.data)     # Convert text to speech
    engine.runAndWait()      # Play the speech

def start_subscriber():
    rospy.init_node('llm_subscriber', anonymous=True)
    rospy.Subscriber('llm_response', String, print_response)
    rospy.spin()

if __name__ == '__main__':
    start_subscriber()

