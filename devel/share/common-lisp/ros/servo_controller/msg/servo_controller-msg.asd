
(cl:in-package :asdf)

(defsystem "servo_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ServoFeedback" :depends-on ("_package_ServoFeedback"))
    (:file "_package_ServoFeedback" :depends-on ("_package"))
  ))