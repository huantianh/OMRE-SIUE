;; Auto-generated. Do not edit!


(when (boundp 'arduino_msgs::RobotInfo)
  (if (not (find-package "ARDUINO_MSGS"))
    (make-package "ARDUINO_MSGS"))
  (shadow 'RobotInfo (find-package "ARDUINO_MSGS")))
(unless (find-package "ARDUINO_MSGS::ROBOTINFO")
  (make-package "ARDUINO_MSGS::ROBOTINFO"))

(in-package "ROS")
;;//! \htmlinclude RobotInfo.msg.html


(defclass arduino_msgs::RobotInfo
  :super ros::object
  :slots (_ultraSonic _ir _enconder ))

(defmethod arduino_msgs::RobotInfo
  (:init
   (&key
    ((:ultraSonic __ultraSonic) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ir __ir) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:enconder __enconder) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _ultraSonic __ultraSonic)
   (setq _ir __ir)
   (setq _enconder __enconder)
   self)
  (:ultraSonic
   (&optional __ultraSonic)
   (if __ultraSonic (setq _ultraSonic __ultraSonic)) _ultraSonic)
  (:ir
   (&optional __ir)
   (if __ir (setq _ir __ir)) _ir)
  (:enconder
   (&optional __enconder)
   (if __enconder (setq _enconder __enconder)) _enconder)
  (:serialization-length
   ()
   (+
    ;; float32[] _ultraSonic
    (* 4    (length _ultraSonic)) 4
    ;; float32[] _ir
    (* 4    (length _ir)) 4
    ;; int32[] _enconder
    (* 4    (length _enconder)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _ultraSonic
     (write-long (length _ultraSonic) s)
     (dotimes (i (length _ultraSonic))
       (sys::poke (elt _ultraSonic i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _ir
     (write-long (length _ir) s)
     (dotimes (i (length _ir))
       (sys::poke (elt _ir i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _enconder
     (write-long (length _enconder) s)
     (dotimes (i (length _enconder))
       (write-long (elt _enconder i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _ultraSonic
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ultraSonic (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ultraSonic i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _ir
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ir (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ir i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _enconder
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _enconder (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _enconder i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get arduino_msgs::RobotInfo :md5sum-) "7a9cbc08a00a37b339035a0f6b3cd87f")
(setf (get arduino_msgs::RobotInfo :datatype-) "arduino_msgs/RobotInfo")
(setf (get arduino_msgs::RobotInfo :definition-)
      "float32[] ultraSonic
float32[] ir
int32[] enconder

")



(provide :arduino_msgs/RobotInfo "7a9cbc08a00a37b339035a0f6b3cd87f")


