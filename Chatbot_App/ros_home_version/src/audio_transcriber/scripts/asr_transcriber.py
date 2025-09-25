#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import UInt8MultiArray, String
from google.cloud import speech
import queue
import threading
import numpy as np

# Set up Google Cloud Speech-to-Text credentials
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "Path_to_your_Google_key.json"

# Initialize Google Cloud Speech client
client = speech.SpeechClient()

# Global queue for buffering incoming audio data
audio_queue = queue.Queue()

# Audio sample rate (Hz)
RATE = 16000

def audio_callback(msg):
    """
    ROS subscriber callback for receiving raw audio messages.
    Incoming audio (UInt8MultiArray) is converted to bytes and 
    placed into the audio processing queue.
    """
    audio_queue.put(bytes(msg.data))

def asr_worker(pub):
    """
    Background worker thread for streaming audio to Google Cloud Speech-to-Text.
    Listens for audio from the queue, sends it to Google ASR, 
    and publishes transcribed text to a ROS topic.
    """
    # Configure recognition parameters
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code="en-US",
        enable_automatic_punctuation=True,
    )

    # Enable streaming recognition
    streaming_config = speech.StreamingRecognitionConfig(
        config=config,
        interim_results=False  # Only publish final results
    )
    
    def request_generator():
        """
        Generator function that yields audio chunks from the queue
        as requests to the Google API until ROS shuts down.
        """
        while not rospy.is_shutdown():
            data = audio_queue.get()
            yield speech.StreamingRecognizeRequest(audio_content=data)
    
    # Start the streaming recognition session
    responses = client.streaming_recognize(streaming_config, request_generator())

    # Continuously process responses from Google Speech API
    while not rospy.is_shutdown():
        try:
            for response in responses:
                for result in response.results:
                    if result.is_final:  # Only handle final transcripts
                        transcript = result.alternatives[0].transcript
                        if transcript != "":
                            rospy.loginfo(f"ASR: {transcript}")
                            pub.publish(transcript)  # Publish transcript to ROS topic
        except Exception as e:
            rospy.logerr(e)  # Log any recognition errors

def asr_transcriber():
    """
    Initializes the ROS node for ASR transcription.
    Subscribes to raw audio topic, publishes transcripts,
    and launches the ASR worker in a background thread.
    """
    rospy.init_node('asr_transcriber', anonymous=True)

    # Publisher for transcribed text
    pub = rospy.Publisher('/asr_transcript', String, queue_size=10)

    # Subscriber to the raw audio stream
    rospy.Subscriber('/audio_data', UInt8MultiArray, audio_callback)
    
    # Start the ASR worker in a separate thread
    asr_thread = threading.Thread(target=asr_worker, args=(pub,))
    asr_thread.start()
    
    # Keep the node alive until shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        asr_transcriber()
    except rospy.ROSInterruptException:
        pass  # Clean shutdown when ROS is stopped
