
(cl:in-package :asdf)

(defsystem "respeaker_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RawAudioData" :depends-on ("_package_RawAudioData"))
    (:file "_package_RawAudioData" :depends-on ("_package"))
  ))