; Auto-generated. Do not edit!


(cl:in-package servo_controller-msg)


;//! \htmlinclude ServoFeedback.msg.html

(cl:defclass <ServoFeedback> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (load
    :reader load
    :initarg :load
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (temper
    :reader temper
    :initarg :temper
    :type cl:float
    :initform 0.0)
   (move
    :reader move
    :initarg :move
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ServoFeedback (<ServoFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_controller-msg:<ServoFeedback> is deprecated: use servo_controller-msg:ServoFeedback instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:id-val is deprecated.  Use servo_controller-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:pos-val is deprecated.  Use servo_controller-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:speed-val is deprecated.  Use servo_controller-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:current-val is deprecated.  Use servo_controller-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:load-val is deprecated.  Use servo_controller-msg:load instead.")
  (load m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:voltage-val is deprecated.  Use servo_controller-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'temper-val :lambda-list '(m))
(cl:defmethod temper-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:temper-val is deprecated.  Use servo_controller-msg:temper instead.")
  (temper m))

(cl:ensure-generic-function 'move-val :lambda-list '(m))
(cl:defmethod move-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:move-val is deprecated.  Use servo_controller-msg:move instead.")
  (move m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_controller-msg:success-val is deprecated.  Use servo_controller-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoFeedback>) ostream)
  "Serializes a message object of type '<ServoFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'move) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoFeedback>) istream)
  "Deserializes a message object of type '<ServoFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'load) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temper) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'move) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoFeedback>)))
  "Returns string type for a message object of type '<ServoFeedback>"
  "servo_controller/ServoFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoFeedback)))
  "Returns string type for a message object of type 'ServoFeedback"
  "servo_controller/ServoFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoFeedback>)))
  "Returns md5sum for a message object of type '<ServoFeedback>"
  "1a521aa655c317a35ffe9a7574584938")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoFeedback)))
  "Returns md5sum for a message object of type 'ServoFeedback"
  "1a521aa655c317a35ffe9a7574584938")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoFeedback>)))
  "Returns full string definition for message of type '<ServoFeedback>"
  (cl:format cl:nil "int32 id~%float32 pos~%float32 speed~%float32 current~%float32 load~%float32 voltage~%float32 temper~%bool move~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoFeedback)))
  "Returns full string definition for message of type 'ServoFeedback"
  (cl:format cl:nil "int32 id~%float32 pos~%float32 speed~%float32 current~%float32 load~%float32 voltage~%float32 temper~%bool move~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoFeedback>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoFeedback
    (cl:cons ':id (id msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':current (current msg))
    (cl:cons ':load (load msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':temper (temper msg))
    (cl:cons ':move (move msg))
    (cl:cons ':success (success msg))
))
