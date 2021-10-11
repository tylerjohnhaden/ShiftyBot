;; Auto-generated. Do not edit!


(when (boundp 'lab1::MapperMessage)
  (if (not (find-package "LAB1"))
    (make-package "LAB1"))
  (shadow 'MapperMessage (find-package "LAB1")))
(unless (find-package "LAB1::MAPPERMESSAGE")
  (make-package "LAB1::MAPPERMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude MapperMessage.msg.html


(defclass lab1::MapperMessage
  :super ros::object
  :slots (_x _y _theta _d ))

(defmethod lab1::MapperMessage
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:theta __theta) 0)
    ((:d __d) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _theta (round __theta))
   (setq _d (round __d))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:serialization-length
   ()
   (+
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _theta
    4
    ;; int32 _d
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _theta
       (write-long _theta s)
     ;; int32 _d
       (write-long _d s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _theta
     (setq _theta (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _d
     (setq _d (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lab1::MapperMessage :md5sum-) "20b51b83ff10e216b97e63c312c297ed")
(setf (get lab1::MapperMessage :datatype-) "lab1/MapperMessage")
(setf (get lab1::MapperMessage :definition-)
      "int32 x
int32 y
int32 theta
int32 d

")



(provide :lab1/MapperMessage "20b51b83ff10e216b97e63c312c297ed")


