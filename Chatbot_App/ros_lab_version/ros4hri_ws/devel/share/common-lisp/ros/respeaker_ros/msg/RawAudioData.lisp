; Auto-generated. Do not edit!


(cl:in-package respeaker_ros-msg)


;//! \htmlinclude RawAudioData.msg.html

(cl:defclass <RawAudioData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (nb_channel
    :reader nb_channel
    :initarg :nb_channel
    :type cl:fixnum
    :initform 0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:integer
    :initform 0)
   (format
    :reader format
    :initarg :format
    :type cl:integer
    :initform 0)
   (sample_byte_size
    :reader sample_byte_size
    :initarg :sample_byte_size
    :type cl:integer
    :initform 0)
   (nb_frames
    :reader nb_frames
    :initarg :nb_frames
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawAudioData (<RawAudioData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawAudioData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawAudioData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name respeaker_ros-msg:<RawAudioData> is deprecated: use respeaker_ros-msg:RawAudioData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:header-val is deprecated.  Use respeaker_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'nb_channel-val :lambda-list '(m))
(cl:defmethod nb_channel-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:nb_channel-val is deprecated.  Use respeaker_ros-msg:nb_channel instead.")
  (nb_channel m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:rate-val is deprecated.  Use respeaker_ros-msg:rate instead.")
  (rate m))

(cl:ensure-generic-function 'format-val :lambda-list '(m))
(cl:defmethod format-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:format-val is deprecated.  Use respeaker_ros-msg:format instead.")
  (format m))

(cl:ensure-generic-function 'sample_byte_size-val :lambda-list '(m))
(cl:defmethod sample_byte_size-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:sample_byte_size-val is deprecated.  Use respeaker_ros-msg:sample_byte_size instead.")
  (sample_byte_size m))

(cl:ensure-generic-function 'nb_frames-val :lambda-list '(m))
(cl:defmethod nb_frames-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:nb_frames-val is deprecated.  Use respeaker_ros-msg:nb_frames instead.")
  (nb_frames m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <RawAudioData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader respeaker_ros-msg:data-val is deprecated.  Use respeaker_ros-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawAudioData>) ostream)
  "Serializes a message object of type '<RawAudioData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'nb_channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'format)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sample_byte_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'nb_frames)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawAudioData>) istream)
  "Deserializes a message object of type '<RawAudioData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nb_channel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'format) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sample_byte_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nb_frames) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawAudioData>)))
  "Returns string type for a message object of type '<RawAudioData>"
  "respeaker_ros/RawAudioData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawAudioData)))
  "Returns string type for a message object of type 'RawAudioData"
  "respeaker_ros/RawAudioData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawAudioData>)))
  "Returns md5sum for a message object of type '<RawAudioData>"
  "1c397debf40a72ecc26b4cfd85d2a668")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawAudioData)))
  "Returns md5sum for a message object of type 'RawAudioData"
  "1c397debf40a72ecc26b4cfd85d2a668")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawAudioData>)))
  "Returns full string definition for message of type '<RawAudioData>"
  (cl:format cl:nil "Header header~%int8   nb_channel~%int32  rate~%int32  format~%int32  sample_byte_size~%int32  nb_frames~%int16[] data~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawAudioData)))
  "Returns full string definition for message of type 'RawAudioData"
  (cl:format cl:nil "Header header~%int8   nb_channel~%int32  rate~%int32  format~%int32  sample_byte_size~%int32  nb_frames~%int16[] data~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawAudioData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawAudioData>))
  "Converts a ROS message object to a list"
  (cl:list 'RawAudioData
    (cl:cons ':header (header msg))
    (cl:cons ':nb_channel (nb_channel msg))
    (cl:cons ':rate (rate msg))
    (cl:cons ':format (format msg))
    (cl:cons ':sample_byte_size (sample_byte_size msg))
    (cl:cons ':nb_frames (nb_frames msg))
    (cl:cons ':data (data msg))
))
