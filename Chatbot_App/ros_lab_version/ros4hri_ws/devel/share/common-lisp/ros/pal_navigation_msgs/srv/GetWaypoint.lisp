; Auto-generated. Do not edit!


(cl:in-package pal_navigation_msgs-srv)


;//! \htmlinclude GetWaypoint-request.msg.html

(cl:defclass <GetWaypoint-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetWaypoint-request (<GetWaypoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_navigation_msgs-srv:<GetWaypoint-request> is deprecated: use pal_navigation_msgs-srv:GetWaypoint-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypoint-request>) ostream)
  "Serializes a message object of type '<GetWaypoint-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypoint-request>) istream)
  "Deserializes a message object of type '<GetWaypoint-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypoint-request>)))
  "Returns string type for a service object of type '<GetWaypoint-request>"
  "pal_navigation_msgs/GetWaypointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoint-request)))
  "Returns string type for a service object of type 'GetWaypoint-request"
  "pal_navigation_msgs/GetWaypointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypoint-request>)))
  "Returns md5sum for a message object of type '<GetWaypoint-request>"
  "a7a8a459ce4c4b9d51acc19ee16d34fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypoint-request)))
  "Returns md5sum for a message object of type 'GetWaypoint-request"
  "a7a8a459ce4c4b9d51acc19ee16d34fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypoint-request>)))
  "Returns full string definition for message of type '<GetWaypoint-request>"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypoint-request)))
  "Returns full string definition for message of type 'GetWaypoint-request"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypoint-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypoint-request
))
;//! \htmlinclude GetWaypoint-response.msg.html

(cl:defclass <GetWaypoint-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type pal_navigation_msgs-msg:Waypoint
    :initform (cl:make-instance 'pal_navigation_msgs-msg:Waypoint)))
)

(cl:defclass GetWaypoint-response (<GetWaypoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_navigation_msgs-srv:<GetWaypoint-response> is deprecated: use pal_navigation_msgs-srv:GetWaypoint-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetWaypoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_navigation_msgs-srv:pose-val is deprecated.  Use pal_navigation_msgs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypoint-response>) ostream)
  "Serializes a message object of type '<GetWaypoint-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypoint-response>) istream)
  "Deserializes a message object of type '<GetWaypoint-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypoint-response>)))
  "Returns string type for a service object of type '<GetWaypoint-response>"
  "pal_navigation_msgs/GetWaypointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoint-response)))
  "Returns string type for a service object of type 'GetWaypoint-response"
  "pal_navigation_msgs/GetWaypointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypoint-response>)))
  "Returns md5sum for a message object of type '<GetWaypoint-response>"
  "a7a8a459ce4c4b9d51acc19ee16d34fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypoint-response)))
  "Returns md5sum for a message object of type 'GetWaypoint-response"
  "a7a8a459ce4c4b9d51acc19ee16d34fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypoint-response>)))
  "Returns full string definition for message of type '<GetWaypoint-response>"
  (cl:format cl:nil "# Response~%pal_navigation_msgs/Waypoint pose~%~%================================================================================~%MSG: pal_navigation_msgs/Waypoint~%# Error codes~%# Note: The expected priority order of the errors should match the message order~%uint32 NOTHING=0~%uint32 WAIT=1~%uint32 ROTATE=2~%uint32 DOCK=3~%uint32 UNDOCK=4~%~%uint32[] actions~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypoint-response)))
  "Returns full string definition for message of type 'GetWaypoint-response"
  (cl:format cl:nil "# Response~%pal_navigation_msgs/Waypoint pose~%~%================================================================================~%MSG: pal_navigation_msgs/Waypoint~%# Error codes~%# Note: The expected priority order of the errors should match the message order~%uint32 NOTHING=0~%uint32 WAIT=1~%uint32 ROTATE=2~%uint32 DOCK=3~%uint32 UNDOCK=4~%~%uint32[] actions~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypoint-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypoint-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWaypoint)))
  'GetWaypoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWaypoint)))
  'GetWaypoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoint)))
  "Returns string type for a service object of type '<GetWaypoint>"
  "pal_navigation_msgs/GetWaypoint")