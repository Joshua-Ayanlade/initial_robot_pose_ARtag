; Auto-generated. Do not edit!


(cl:in-package mobile_robot-srv)


;//! \htmlinclude amclPose-request.msg.html

(cl:defclass <amclPose-request> (roslisp-msg-protocol:ros-message)
  ((getPose
    :reader getPose
    :initarg :getPose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass amclPose-request (<amclPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amclPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amclPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobile_robot-srv:<amclPose-request> is deprecated: use mobile_robot-srv:amclPose-request instead.")))

(cl:ensure-generic-function 'getPose-val :lambda-list '(m))
(cl:defmethod getPose-val ((m <amclPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobile_robot-srv:getPose-val is deprecated.  Use mobile_robot-srv:getPose instead.")
  (getPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amclPose-request>) ostream)
  "Serializes a message object of type '<amclPose-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'getPose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amclPose-request>) istream)
  "Deserializes a message object of type '<amclPose-request>"
    (cl:setf (cl:slot-value msg 'getPose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amclPose-request>)))
  "Returns string type for a service object of type '<amclPose-request>"
  "mobile_robot/amclPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amclPose-request)))
  "Returns string type for a service object of type 'amclPose-request"
  "mobile_robot/amclPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amclPose-request>)))
  "Returns md5sum for a message object of type '<amclPose-request>"
  "93e35a8b398245d681b8a701db81ea2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amclPose-request)))
  "Returns md5sum for a message object of type 'amclPose-request"
  "93e35a8b398245d681b8a701db81ea2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amclPose-request>)))
  "Returns full string definition for message of type '<amclPose-request>"
  (cl:format cl:nil "bool getPose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amclPose-request)))
  "Returns full string definition for message of type 'amclPose-request"
  (cl:format cl:nil "bool getPose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amclPose-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amclPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'amclPose-request
    (cl:cons ':getPose (getPose msg))
))
;//! \htmlinclude amclPose-response.msg.html

(cl:defclass <amclPose-response> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass amclPose-response (<amclPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <amclPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'amclPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobile_robot-srv:<amclPose-response> is deprecated: use mobile_robot-srv:amclPose-response instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <amclPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobile_robot-srv:pos_x-val is deprecated.  Use mobile_robot-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <amclPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobile_robot-srv:pos_y-val is deprecated.  Use mobile_robot-srv:pos_y instead.")
  (pos_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <amclPose-response>) ostream)
  "Serializes a message object of type '<amclPose-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <amclPose-response>) istream)
  "Deserializes a message object of type '<amclPose-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<amclPose-response>)))
  "Returns string type for a service object of type '<amclPose-response>"
  "mobile_robot/amclPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amclPose-response)))
  "Returns string type for a service object of type 'amclPose-response"
  "mobile_robot/amclPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<amclPose-response>)))
  "Returns md5sum for a message object of type '<amclPose-response>"
  "93e35a8b398245d681b8a701db81ea2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'amclPose-response)))
  "Returns md5sum for a message object of type 'amclPose-response"
  "93e35a8b398245d681b8a701db81ea2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<amclPose-response>)))
  "Returns full string definition for message of type '<amclPose-response>"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'amclPose-response)))
  "Returns full string definition for message of type 'amclPose-response"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <amclPose-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <amclPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'amclPose-response
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'amclPose)))
  'amclPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'amclPose)))
  'amclPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'amclPose)))
  "Returns string type for a service object of type '<amclPose>"
  "mobile_robot/amclPose")