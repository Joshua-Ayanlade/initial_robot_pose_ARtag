
(cl:in-package :asdf)

(defsystem "mobile_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "amclPose" :depends-on ("_package_amclPose"))
    (:file "_package_amclPose" :depends-on ("_package"))
  ))