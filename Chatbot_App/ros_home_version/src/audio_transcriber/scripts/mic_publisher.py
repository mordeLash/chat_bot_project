#!/usr/bin/env python3
import rospy
import pyaudio
from std_msgs.msg import UInt8MultiArray

# Audio recording parameters
RATE = 16000  # Sampling rate (Hz)
CHUNK = int(RATE / 10)  # Size of audio buffer per read (100ms of audio)

def mic_publisher():
    """
    ROS node that captures audio from the system microphone using PyAudio
    and publishes it as UInt8MultiArray messages to the '/audio_data' topic.
    """
    # Initialize ROS node
    rospy.init_node('mic_publisher', anonymous=True)

    # Publisher for streaming raw audio data
    pub = rospy.Publisher('/audio_data', UInt8MultiArray, queue_size=10)
    
    # Initialize PyAudio and open the microphone stream
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,      # 16-bit PCM
        channels=1,                  # Mono channel
        rate=RATE,                   # Sample rate
        input=True,                  # Set as input (microphone)
        frames_per_buffer=CHUNK      # Buffer size
    )
    
    rospy.loginfo("Microphone publisher started")
    
    # Continuously capture and publish audio until ROS shutdown
    while not rospy.is_shutdown():
        data = stream.read(CHUNK, exception_on_overflow=False)  # Read 100ms of audio
        msg = UInt8MultiArray(data=bytearray(data))  # Convert raw bytes into ROS message
        pub.publish(msg)

if __name__ == '__main__':
    try:
        mic_publisher()
    except rospy.ROSInterruptException:
        pass  # Graceful shutdown on ROS interrupt
