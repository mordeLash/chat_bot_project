;; Auto-generated. Do not edit!


(when (boundp 'respeaker_ros::RawAudioData)
  (if (not (find-package "RESPEAKER_ROS"))
    (make-package "RESPEAKER_ROS"))
  (shadow 'RawAudioData (find-package "RESPEAKER_ROS")))
(unless (find-package "RESPEAKER_ROS::RAWAUDIODATA")
  (make-package "RESPEAKER_ROS::RAWAUDIODATA"))

(in-package "ROS")
;;//! \htmlinclude RawAudioData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass respeaker_ros::RawAudioData
  :super ros::object
  :slots (_header _nb_channel _rate _format _sample_byte_size _nb_frames _data ))

(defmethod respeaker_ros::RawAudioData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:nb_channel __nb_channel) 0)
    ((:rate __rate) 0)
    ((:format __format) 0)
    ((:sample_byte_size __sample_byte_size) 0)
    ((:nb_frames __nb_frames) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _nb_channel (round __nb_channel))
   (setq _rate (round __rate))
   (setq _format (round __format))
   (setq _sample_byte_size (round __sample_byte_size))
   (setq _nb_frames (round __nb_frames))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:nb_channel
   (&optional __nb_channel)
   (if __nb_channel (setq _nb_channel __nb_channel)) _nb_channel)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:format
   (&optional __format)
   (if __format (setq _format __format)) _format)
  (:sample_byte_size
   (&optional __sample_byte_size)
   (if __sample_byte_size (setq _sample_byte_size __sample_byte_size)) _sample_byte_size)
  (:nb_frames
   (&optional __nb_frames)
   (if __nb_frames (setq _nb_frames __nb_frames)) _nb_frames)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _nb_channel
    1
    ;; int32 _rate
    4
    ;; int32 _format
    4
    ;; int32 _sample_byte_size
    4
    ;; int32 _nb_frames
    4
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _nb_channel
       (write-byte _nb_channel s)
     ;; int32 _rate
       (write-long _rate s)
     ;; int32 _format
       (write-long _format s)
     ;; int32 _sample_byte_size
       (write-long _sample_byte_size s)
     ;; int32 _nb_frames
       (write-long _nb_frames s)
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _nb_channel
     (setq _nb_channel (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _nb_channel 127) (setq _nb_channel (- _nb_channel 256)))
   ;; int32 _rate
     (setq _rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _format
     (setq _format (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sample_byte_size
     (setq _sample_byte_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _nb_frames
     (setq _nb_frames (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get respeaker_ros::RawAudioData :md5sum-) "1c397debf40a72ecc26b4cfd85d2a668")
(setf (get respeaker_ros::RawAudioData :datatype-) "respeaker_ros/RawAudioData")
(setf (get respeaker_ros::RawAudioData :definition-)
      "Header header
int8   nb_channel
int32  rate
int32  format
int32  sample_byte_size
int32  nb_frames
int16[] data



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :respeaker_ros/RawAudioData "1c397debf40a72ecc26b4cfd85d2a668")


