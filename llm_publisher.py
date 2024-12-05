#!/usr/bin/env python3
# llm_publisher.py

import rospy
import requests
import pyttsx3  # For text-to-speech
import speech_recognition as sr  # For speech-to-text
from std_msgs.msg import String

API_URL = "https://api-inference.huggingface.co/models/bigscience/bloom"
headers = {"Authorization": "Bearer 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"}

# 1. Enhanced Error Handling for API Queries
def query_huggingface(prompt):
    for attempt in range(3):  # Retry mechanism
        try:
            response = requests.post(API_URL, headers=headers, json={"inputs": prompt})
            response.raise_for_status()  # Raise HTTPError for bad responses
            return response.json()[0]["generated_text"]
        except requests.exceptions.HTTPError as e:
            rospy.logerr(f"HTTP Error: {e} - Attempt {attempt + 1}")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Request Exception: {e} - Attempt {attempt + 1}")
    return "Error: Could not fetch a response from the API after 3 attempts."

# 2. Error Handling for Text-to-Speech
def text_to_speech(text):
    try:
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
        engine.stop()
    except Exception as e:
        rospy.logwarn(f"Text-to-Speech error: {e}")
        print("Unable to use text-to-speech. Please read the response on the screen.")

# 3. Error Handling for Speech-to-Text
def speech_to_text():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for your question...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        try:
            audio = recognizer.listen(source, timeout=10)  # Timeout after 10 seconds
            question = recognizer.recognize_google(audio)
            print(f"You said: {question}")
            return question
        except sr.UnknownValueError:
            print("Sorry, I couldn't understand your speech. Please try again.")
            return None
        except sr.RequestError as e:
            print(f"Speech Recognition error: {e}")
            return None

# 4. ROS Initialization with Error Handling
def initialize_ros():
    try:
        rospy.init_node('llm_publisher', anonymous=True)
        pub = rospy.Publisher('llm_response', String, queue_size=10)
        return pub
    except rospy.ROSException as e:
        rospy.logerr(f"Failed to initialize ROS node or publisher: {e}")
        return None

def llm_publisher():
    pub = initialize_ros()
    if not pub:  # Exit if ROS initialization failed
        print("ROS initialization failed. Exiting program.")
        return

    input_mode = input("Choose input mode: 'text' for typing, 'speech' for voice input: ").strip()
    output_mode = input("Choose output mode: 'text' for text, 'speech' for voice output: ").strip()

    while not rospy.is_shutdown():
        # 5. Stopping Criteria
        question = None

        if input_mode == "speech":
            question = speech_to_text()
            if question and question.lower() in ['exit', 'quit', 'stop']:  # Exit criteria
                print("Exiting program. Goodbye!")
                break
            if not question:
                continue
	
        elif input_mode == "text":
            question = input("Enter your question (type 'exit' to quit): ").strip()
            if question.lower() in ['exit', 'quit', 'stop']:  # Exit criteria
                print("Exiting program. Goodbye!")
                break

        if question:
            response = query_huggingface(question)
            rospy.loginfo(f"Response: {response}")
            pub.publish(response)

            if output_mode == "speech":
                text_to_speech(response)
            elif output_mode == "text":
                print(response)

        # Allow user to switch modes or exit
        try:
            switch_mode = input("Switch modes? Type 'input' or 'output' to switch, or 'no' to continue: ").strip()
            if switch_mode == "input":
                input_mode = input("Choose input mode: 'text' or 'speech': ").strip()
            elif switch_mode == "output":
                output_mode = input("Choose output mode: 'text' or 'speech': ").strip()
        except KeyboardInterrupt:
            print("\nProgram interrupted. Exiting.")
            break

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        llm_publisher()
    except rospy.ROSInterruptException:
        print("ROS Interrupted. Exiting.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

