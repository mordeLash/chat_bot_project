#!/usr/bin/env python3
# File: src/audio_transcriber/scripts/llm_node.py

import os
import sys
import rospkg

# get the absolute path to this package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path("audio_transcriber")

# add package and subfolder to sys.path
sys.path.append(os.path.join(pkg_path))
sys.path.append(os.path.join(pkg_path, "rag_llm"))


import rospy
from std_msgs.msg import String
from rag_llm.llm_app import LLMApp
from colorama import Fore, Style, init

# Initialize colorama for colored console output
init(autoreset=True)

class LLMNode:
    """
    ROS node that connects speech recognition (ASR) output to a language model (LLM),
    and sends the LLM’s response to the text-to-speech (TTS) system.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('llm_node', anonymous=True)

        # Load parameters from ROS parameter server
        model = rospy.get_param("~model_name")   # LLM model name (e.g., "llama3.1")
        character = rospy.get_param("~character")  # Personality/character settings for the chatbot

        # Initialize LLM application wrapper
        self.llm_app = LLMApp(model=model, num_predict=500, character=character)

        # Subscribe to ASR transcript topic
        rospy.Subscriber("/asr_transcript", String, self.transcription_callback)

        rospy.loginfo("LLM Node is ready and waiting for transcriptions.")




    def text_publisher(self, text):
        """function to get simple TTS at
        home to replace built in PAL TTS"""
        from espeakng import ESpeakNG
        # Initialize engine
        esng = ESpeakNG()
        # Choose voice (list available with: esng.voices)
        esng.voice = 'en-us'  
        esng.speed = 150        # slower speech rate (default ~175)
        esng.pitch = 40         # lower pitch = less robotic (default ~50)
        esng.amplitude = 150    # volume (default 100, max 200)
        # Speak text
        esng.say(text)

    def transcription_callback(self, msg):
        """
        Callback for processing transcriptions from ASR.
        Sends text to LLM, prints response to console,
        and forwards answer to TTS system.
        """
        transcription = msg.data
        print(Fore.LIGHTWHITE_EX + "\n🧑 User:", transcription)

        print(Fore.LIGHTWHITE_EX + "\n🤖 Assistant: ", end="", flush=True)

        # Generate complete LLM response
        answer = self.llm_app.chat(transcription, log_to_file=True)

        # Print assistant’s response in console
        print(answer)
        print()  # Extra newline for readability

        # Send response to TTS system
        self.text_publisher(answer)

if __name__ == '__main__':
    try:
        # Start LLM node
        LLMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  # Graceful shutdown on ROS interrupt
