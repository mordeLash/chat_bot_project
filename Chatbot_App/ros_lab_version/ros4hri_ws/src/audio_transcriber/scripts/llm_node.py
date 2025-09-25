#!/usr/bin/env python3
# File: src/audio_transcriber/scripts/llm_node.py

import sys
import os
import rospkg
from pal_interaction_msgs.msg import TtsActionGoal

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
    ROS node that connects ASR (speech recognition) transcriptions
    to a language model (LLM), and forwards the generated responses
    to the TTS (text-to-speech) system.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('llm_node', anonymous=True)

        # Load parameters from ROS parameter server
        model = rospy.get_param("~model_name")      # e.g., "llama3.1"
        character = rospy.get_param("~character")  # chatbot character/personality
        rag = False
        if(character=="signal_processing_expert"):
            rag=True
        # Initialize the LLM application wrapper
        rospy.loginfo(f"model is {model}, character is {character}")
        self.llm_app = LLMApp(model=model, num_predict=500, character=character,use_rag=rag)

        # Subscribe to ASR transcription topic
        rospy.Subscriber("/asr_transcript", String, self.transcription_callback)

        # Publisher for sending responses to PAL Robotics’ TTS system
        self.pub = rospy.Publisher('/tts/goal', TtsActionGoal, latch=True, queue_size=1)

        rospy.loginfo("LLM Node is ready and waiting for transcriptions.")

    def text_publisher(self, text):
        """
        Publish text as a TTS goal to the PAL Robotics TTS system.
        Waits until there is at least one subscriber before publishing.
        """
        msg = TtsActionGoal()
        msg.goal.rawtext.text = text
        msg.goal.rawtext.lang_id = 'en_GB'  # British English voice

        # Ensure TTS node is connected before publishing
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # Publish message to TTS
        self.pub.publish(msg)

        # Small delay to allow TTS system to process
        rospy.sleep(0.5)

    def transcription_callback(self, msg):
        """
        Callback for handling transcriptions from ASR.
        - Logs the user’s transcription
        - Passes it to the LLM
        - Prints the LLM’s response in the console
        - Publishes the response to the TTS system
        """
        transcription = msg.data

        # Print user input
        print(Fore.LIGHTWHITE_EX + "\n🧑 User:", transcription)

        # Print assistant response (stream-like formatting)
        print(Fore.LIGHTWHITE_EX + "\n🤖 Assistant: ", end="", flush=True)

        # Generate assistant response (full response returned as string)
        answer = self.llm_app.chat(transcription, log_to_file=True)

        # Print assistant response
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
        pass  # Graceful shutdown when ROS stops
