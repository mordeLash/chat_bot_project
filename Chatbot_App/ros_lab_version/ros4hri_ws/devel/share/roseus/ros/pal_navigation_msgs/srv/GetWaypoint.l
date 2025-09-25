;; Auto-generated. Do not edit!


(when (boundp 'pal_navigation_msgs::GetWaypoint)
  (if (not (find-package "PAL_NAVIGATION_MSGS"))
    (make-package "PAL_NAVIGATION_MSGS"))
  (shadow 'GetWaypoint (find-package "PAL_NAVIGATION_MSGS")))
(unless (find-package "PAL_NAVIGATION_MSGS::GETWAYPOINT")
  (make-package "PAL_NAVIGATION_MSGS::GETWAYPOINT"))
(unless (find-package "PAL_NAVIGATION_MSGS::GETWAYPOINTREQUEST")
  (make-package "PAL_NAVIGATION_MSGS::GETWAYPOINTREQUEST"))
(unless (find-package "PAL_NAVIGATION_MSGS::GETWAYPOINTRESPONSE")
  (make-package "PAL_NAVIGATION_MSGS::GETWAYPOINTRESPONSE"))

(in-package "ROS")





(defclass pal_navigation_msgs::GetWaypointRequest
  :super ros::object
  :slots ())

(defmethod pal_navigation_msgs::GetWaypointRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass pal_navigation_msgs::GetWaypointResponse
  :super ros::object
  :slots (_pose ))

(defmethod pal_navigation_msgs::GetWaypointResponse
  (:init
   (&key
    ((:pose __pose) (instance pal_navigation_msgs::Waypoint :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; pal_navigation_msgs/Waypoint _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pal_navigation_msgs/Waypoint _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pal_navigation_msgs/Waypoint _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass pal_navigation_msgs::GetWaypoint
  :super ros::object
  :slots ())

(setf (get pal_navigation_msgs::GetWaypoint :md5sum-) "a7a8a459ce4c4b9d51acc19ee16d34fd")
(setf (get pal_navigation_msgs::GetWaypoint :datatype-) "pal_navigation_msgs/GetWaypoint")
(setf (get pal_navigation_msgs::GetWaypoint :request) pal_navigation_msgs::GetWaypointRequest)
(setf (get pal_navigation_msgs::GetWaypoint :response) pal_navigation_msgs::GetWaypointResponse)

(defmethod pal_navigation_msgs::GetWaypointRequest
  (:response () (instance pal_navigation_msgs::GetWaypointResponse :init)))

(setf (get pal_navigation_msgs::GetWaypointRequest :md5sum-) "a7a8a459ce4c4b9d51acc19ee16d34fd")
(setf (get pal_navigation_msgs::GetWaypointRequest :datatype-) "pal_navigation_msgs/GetWaypointRequest")
(setf (get pal_navigation_msgs::GetWaypointRequest :definition-)
      "# Request
---
# Response
pal_navigation_msgs/Waypoint pose

================================================================================
MSG: pal_navigation_msgs/Waypoint
# Error codes
# Note: The expected priority order of the errors should match the message order
uint32 NOTHING=0
uint32 WAIT=1
uint32 ROTATE=2
uint32 DOCK=3
uint32 UNDOCK=4

uint32[] actions
geometry_msgs/PoseStamped pose
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get pal_navigation_msgs::GetWaypointResponse :md5sum-) "a7a8a459ce4c4b9d51acc19ee16d34fd")
(setf (get pal_navigation_msgs::GetWaypointResponse :datatype-) "pal_navigation_msgs/GetWaypointResponse")
(setf (get pal_navigation_msgs::GetWaypointResponse :definition-)
      "# Request
---
# Response
pal_navigation_msgs/Waypoint pose

================================================================================
MSG: pal_navigation_msgs/Waypoint
# Error codes
# Note: The expected priority order of the errors should match the message order
uint32 NOTHING=0
uint32 WAIT=1
uint32 ROTATE=2
uint32 DOCK=3
uint32 UNDOCK=4

uint32[] actions
geometry_msgs/PoseStamped pose
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :pal_navigation_msgs/GetWaypoint "a7a8a459ce4c4b9d51acc19ee16d34fd")


