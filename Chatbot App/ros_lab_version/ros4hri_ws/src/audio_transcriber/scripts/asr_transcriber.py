#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import String
from google.cloud import speech
import queue
import threading
import numpy as np
from respeaker_ros.msg import RawAudioData

# Set up Google Cloud Speech-to-Text credentials
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/ros/chatbot_project/Google_key.json"

# Initialize Google Cloud Speech client
client = speech.SpeechClient()

# Global queue for buffering processed audio data
audio_queue = queue.Queue()

# Audio sample rate (Hz)
RATE = 16000


def audio_callback(msg):
    """
    ROS subscriber callback for receiving raw multi-channel audio messages.

    - Converts incoming audio from UInt8MultiArray to int16 numpy array
    - Extracts:
        * Channel 1 (index 1) as the main speech signal
        * Channel 5 (index 5) as the "ARI voice" reference
    - Uses the ARI channel to determine whether to mute the main channel
    - Pushes processed audio bytes into the audio_queue for ASR
    """
    multi_ch = msg.data
    # Reshape flat array into [frames=512, channels=6]
    single_ch = np.array(multi_ch, dtype=np.int16).reshape(512, 6)[:, 1]  # main speech channel
    ari_voice = np.array(multi_ch, dtype=np.int16).reshape(512, 6)[:, 5]  # ARI reference channel

    # If ARI channel is active (more than one unique value), mute main channel
    if len(np.unique(ari_voice)) > 1:
        gain = 0
    else:
        gain = 1
    single_ch *= gain

    # Push audio to queue for ASR worker
    audio_queue.put(single_ch.tobytes())


def asr_worker(pub):
    """
    Background worker thread for streaming audio to Google Cloud Speech-to-Text.
    - Reads audio chunks from queue
    - Sends chunks as streaming requests
    - Publishes transcriptions to a ROS topic
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
        Generator that yields audio chunks from the queue
        to the Google Speech-to-Text API until ROS shuts down.
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
    - Subscribes to the raw audio topic
    - Publishes recognized transcripts
    - Runs the ASR worker in a background thread
    """
    rospy.init_node('asr_transcriber', anonymous=True)

    # Publisher for transcribed text
    pub = rospy.Publisher('/asr_transcript', String, queue_size=10)

    # Subscriber to raw multi-channel audio
    rospy.Subscriber('/audio/raw_audio', RawAudioData, audio_callback)

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
