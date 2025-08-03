;; Auto-generated. Do not edit!


(when (boundp 'servo_controller::ServoFeedback)
  (if (not (find-package "SERVO_CONTROLLER"))
    (make-package "SERVO_CONTROLLER"))
  (shadow 'ServoFeedback (find-package "SERVO_CONTROLLER")))
(unless (find-package "SERVO_CONTROLLER::SERVOFEEDBACK")
  (make-package "SERVO_CONTROLLER::SERVOFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude ServoFeedback.msg.html


(defclass servo_controller::ServoFeedback
  :super ros::object
  :slots (_id _pos _speed _current _load _voltage _temper _move _success ))

(defmethod servo_controller::ServoFeedback
  (:init
   (&key
    ((:id __id) 0)
    ((:pos __pos) 0.0)
    ((:speed __speed) 0.0)
    ((:current __current) 0.0)
    ((:load __load) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temper __temper) 0.0)
    ((:move __move) nil)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pos (float __pos))
   (setq _speed (float __speed))
   (setq _current (float __current))
   (setq _load (float __load))
   (setq _voltage (float __voltage))
   (setq _temper (float __temper))
   (setq _move __move)
   (setq _success __success)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temper
   (&optional __temper)
   (if __temper (setq _temper __temper)) _temper)
  (:move
   (&optional (__move :null))
   (if (not (eq __move :null)) (setq _move __move)) _move)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32 _pos
    4
    ;; float32 _speed
    4
    ;; float32 _current
    4
    ;; float32 _load
    4
    ;; float32 _voltage
    4
    ;; float32 _temper
    4
    ;; bool _move
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float32 _pos
       (sys::poke _pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _load
       (sys::poke _load (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temper
       (sys::poke _temper (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _move
       (if _move (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _pos
     (setq _pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _load
     (setq _load (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temper
     (setq _temper (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _move
     (setq _move (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get servo_controller::ServoFeedback :md5sum-) "1a521aa655c317a35ffe9a7574584938")
(setf (get servo_controller::ServoFeedback :datatype-) "servo_controller/ServoFeedback")
(setf (get servo_controller::ServoFeedback :definition-)
      "int32 id
float32 pos
float32 speed
float32 current
float32 load
float32 voltage
float32 temper
bool move
bool success


")



(provide :servo_controller/ServoFeedback "1a521aa655c317a35ffe9a7574584938")


