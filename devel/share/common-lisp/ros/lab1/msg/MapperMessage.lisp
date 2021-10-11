; Auto-generated. Do not edit!


(cl:in-package lab1-msg)


;//! \htmlinclude MapperMessage.msg.html

(cl:defclass <MapperMessage> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:integer
    :initform 0)
   (d
    :reader d
    :initarg :d
    :type cl:integer
    :initform 0))
)

(cl:defclass MapperMessage (<MapperMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapperMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapperMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab1-msg:<MapperMessage> is deprecated: use lab1-msg:MapperMessage instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <MapperMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab1-msg:x-val is deprecated.  Use lab1-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <MapperMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab1-msg:y-val is deprecated.  Use lab1-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <MapperMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab1-msg:theta-val is deprecated.  Use lab1-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <MapperMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab1-msg:d-val is deprecated.  Use lab1-msg:d instead.")
  (d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapperMessage>) ostream)
  "Serializes a message object of type '<MapperMessage>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'theta)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapperMessage>) istream)
  "Deserializes a message object of type '<MapperMessage>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'theta) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapperMessage>)))
  "Returns string type for a message object of type '<MapperMessage>"
  "lab1/MapperMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapperMessage)))
  "Returns string type for a message object of type 'MapperMessage"
  "lab1/MapperMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapperMessage>)))
  "Returns md5sum for a message object of type '<MapperMessage>"
  "20b51b83ff10e216b97e63c312c297ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapperMessage)))
  "Returns md5sum for a message object of type 'MapperMessage"
  "20b51b83ff10e216b97e63c312c297ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapperMessage>)))
  "Returns full string definition for message of type '<MapperMessage>"
  (cl:format cl:nil "int32 x~%int32 y~%int32 theta~%int32 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapperMessage)))
  "Returns full string definition for message of type 'MapperMessage"
  (cl:format cl:nil "int32 x~%int32 y~%int32 theta~%int32 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapperMessage>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapperMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'MapperMessage
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':d (d msg))
))
