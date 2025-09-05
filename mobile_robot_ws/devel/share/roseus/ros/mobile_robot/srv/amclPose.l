;; Auto-generated. Do not edit!


(when (boundp 'mobile_robot::amclPose)
  (if (not (find-package "MOBILE_ROBOT"))
    (make-package "MOBILE_ROBOT"))
  (shadow 'amclPose (find-package "MOBILE_ROBOT")))
(unless (find-package "MOBILE_ROBOT::AMCLPOSE")
  (make-package "MOBILE_ROBOT::AMCLPOSE"))
(unless (find-package "MOBILE_ROBOT::AMCLPOSEREQUEST")
  (make-package "MOBILE_ROBOT::AMCLPOSEREQUEST"))
(unless (find-package "MOBILE_ROBOT::AMCLPOSERESPONSE")
  (make-package "MOBILE_ROBOT::AMCLPOSERESPONSE"))

(in-package "ROS")





(defclass mobile_robot::amclPoseRequest
  :super ros::object
  :slots (_getPose ))

(defmethod mobile_robot::amclPoseRequest
  (:init
   (&key
    ((:getPose __getPose) nil)
    )
   (send-super :init)
   (setq _getPose __getPose)
   self)
  (:getPose
   (&optional __getPose)
   (if __getPose (setq _getPose __getPose)) _getPose)
  (:serialization-length
   ()
   (+
    ;; bool _getPose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _getPose
       (if _getPose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _getPose
     (setq _getPose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mobile_robot::amclPoseResponse
  :super ros::object
  :slots (_pos_x _pos_y ))

(defmethod mobile_robot::amclPoseResponse
  (:init
   (&key
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    )
   (send-super :init)
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   self)
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:serialization-length
   ()
   (+
    ;; float32 _pos_x
    4
    ;; float32 _pos_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pos_x
     (setq _pos_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pos_y
     (setq _pos_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mobile_robot::amclPose
  :super ros::object
  :slots ())

(setf (get mobile_robot::amclPose :md5sum-) "93e35a8b398245d681b8a701db81ea2e")
(setf (get mobile_robot::amclPose :datatype-) "mobile_robot/amclPose")
(setf (get mobile_robot::amclPose :request) mobile_robot::amclPoseRequest)
(setf (get mobile_robot::amclPose :response) mobile_robot::amclPoseResponse)

(defmethod mobile_robot::amclPoseRequest
  (:response () (instance mobile_robot::amclPoseResponse :init)))

(setf (get mobile_robot::amclPoseRequest :md5sum-) "93e35a8b398245d681b8a701db81ea2e")
(setf (get mobile_robot::amclPoseRequest :datatype-) "mobile_robot/amclPoseRequest")
(setf (get mobile_robot::amclPoseRequest :definition-)
      "bool getPose
---
float32 pos_x
float32 pos_y

")

(setf (get mobile_robot::amclPoseResponse :md5sum-) "93e35a8b398245d681b8a701db81ea2e")
(setf (get mobile_robot::amclPoseResponse :datatype-) "mobile_robot/amclPoseResponse")
(setf (get mobile_robot::amclPoseResponse :definition-)
      "bool getPose
---
float32 pos_x
float32 pos_y

")



(provide :mobile_robot/amclPose "93e35a8b398245d681b8a701db81ea2e")


