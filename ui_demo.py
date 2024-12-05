#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import tkinter as tk
from tkinter import messagebox
import threading
import requests
import pyttsx3
import speech_recognition as sr

# Initialize ROS node and publisher
rospy.init_node('llm_ui_node', anonymous=True)
pub = rospy.Publisher('llm_response', String, queue_size=10)

# API details (replace with valid API credentials)
API_URL = "https://api-inference.huggingface.co/models/bigscience/bloom"
headers = {"Authorization": "Bearer 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"}

# Initialize TTS engine
tts_engine = pyttsx3.init()

def query_huggingface(prompt):
    try:
        response = requests.post(API_URL, headers=headers, json={"inputs": prompt})
        if response.status_code == 200:
            return response.json()[0]["generated_text"]
        else:
            rospy.logerr(f"API Error {response.status_code}: {response.text}")
            return "Error: Unable to fetch response."
    except Exception as e:
        rospy.logerr(f"API request failed: {e}")
        return "Error: API request failed."

def save_to_session(question, response):
    with open("session_log.txt", "a") as file:
        file.write(f"Q: {question}\nA: {response}\n\n")

def handle_query_from_ui():
    question = question_input.get("1.0", "end").strip()  # Fetch multi-line input
    if question:
        response = query_huggingface(question)
        response_text.set(response)
        submitted_question.set(f"Your Question: {question}")  # Show question after submission
        question_input.delete("1.0", "end")  # Clear the question input after submission
        pub.publish(response)
        save_to_session(question, response)
    else:
        messagebox.showwarning("Input Error", "Please enter a valid question.")

def handle_query_from_speech(question):
    response = query_huggingface(question)
    response_text.set(response)
    submitted_question.set(f"Your Question: {question}")  # Show question after submission
    pub.publish(response)
    save_to_session(question, response)

def text_to_speech(text):
    try:
        tts_engine.say(text)
        tts_engine.runAndWait()
    except Exception as e:
        rospy.logwarn(f"TTS error: {e}")

def speech_to_text_thread():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        response_text.set("Listening...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        try:
            audio = recognizer.listen(source, timeout=10)
            question = recognizer.recognize_google(audio)
            response_text.set(f"You said: {question}")
            handle_query_from_speech(question)
        except sr.UnknownValueError:
            response_text.set("Sorry, could not understand. Please try again.")
        except sr.RequestError as e:
            response_text.set(f"Speech Recognition Error: {e}")
        except Exception as e:
            response_text.set(f"Error: {e}")

def start_speech_recognition():
    threading.Thread(target=speech_to_text_thread).start()

def speak_response():
    text_to_speech(response_text.get())

def switch_account():
    messagebox.showinfo("Account Switch", "Feature under construction.")

def on_focus_in(event):
    if question_input.get("1.0", "end-1c") == "How can I help you?":
        question_input.delete("1.0", "end")

def on_focus_out(event):
    if question_input.get("1.0", "end-1c") == "":
        question_input.insert("1.0", "How can I help you?")

# Tkinter UI
root = tk.Tk()
root.title("LLM UI")

# Multi-line Question Input
question_input = tk.Text(root, width=50, height=5, wrap="word")
question_input.insert("1.0", "How can I help you?")  # Set initial placeholder text
question_input.bind("<FocusIn>", on_focus_in)
question_input.bind("<FocusOut>", on_focus_out)
question_input.pack(pady=5)

# Submit Button
tk.Button(root, text="Submit", command=handle_query_from_ui).pack(pady=5)

# Speech Recognition Button
tk.Button(root, text="Speak", command=start_speech_recognition).pack(pady=5)

# Submitted Question Display (at top of answer section)
submitted_question = tk.StringVar()
submitted_label = tk.Label(root, textvariable=submitted_question, wraplength=400, justify="left", bg="lightgray", width=50, height=2)
submitted_label.pack(pady=5)

# Response Display
response_text = tk.StringVar()
response_label = tk.Label(root, textvariable=response_text, wraplength=400, justify="left", bg="lightgray", width=50, height=10)
response_label.pack(pady=10)

# Speak Answer Button
tk.Button(root, text="Speak Answer", command=speak_response).pack(pady=5)

# Account Switch Button
tk.Button(root, text="Switch Account", command=switch_account).pack(pady=5)

# Run the Tkinter main loop
root.mainloop()

