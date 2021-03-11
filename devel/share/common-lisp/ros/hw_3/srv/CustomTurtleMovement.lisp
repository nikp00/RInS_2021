; Auto-generated. Do not edit!


(cl:in-package hw_3-srv)


;//! \htmlinclude CustomTurtleMovement-request.msg.html

(cl:defclass <CustomTurtleMovement-request> (roslisp-msg-protocol:ros-message)
  ((movementType
    :reader movementType
    :initarg :movementType
    :type cl:string
    :initform "")
   (duration
    :reader duration
    :initarg :duration
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CustomTurtleMovement-request (<CustomTurtleMovement-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomTurtleMovement-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomTurtleMovement-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hw_3-srv:<CustomTurtleMovement-request> is deprecated: use hw_3-srv:CustomTurtleMovement-request instead.")))

(cl:ensure-generic-function 'movementType-val :lambda-list '(m))
(cl:defmethod movementType-val ((m <CustomTurtleMovement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hw_3-srv:movementType-val is deprecated.  Use hw_3-srv:movementType instead.")
  (movementType m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <CustomTurtleMovement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hw_3-srv:duration-val is deprecated.  Use hw_3-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomTurtleMovement-request>) ostream)
  "Serializes a message object of type '<CustomTurtleMovement-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'movementType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'movementType))
  (cl:let* ((signed (cl:slot-value msg 'duration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomTurtleMovement-request>) istream)
  "Deserializes a message object of type '<CustomTurtleMovement-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'movementType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'movementType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomTurtleMovement-request>)))
  "Returns string type for a service object of type '<CustomTurtleMovement-request>"
  "hw_3/CustomTurtleMovementRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomTurtleMovement-request)))
  "Returns string type for a service object of type 'CustomTurtleMovement-request"
  "hw_3/CustomTurtleMovementRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomTurtleMovement-request>)))
  "Returns md5sum for a message object of type '<CustomTurtleMovement-request>"
  "f0f2a6af04cef065d9ec88019ed73d34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomTurtleMovement-request)))
  "Returns md5sum for a message object of type 'CustomTurtleMovement-request"
  "f0f2a6af04cef065d9ec88019ed73d34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomTurtleMovement-request>)))
  "Returns full string definition for message of type '<CustomTurtleMovement-request>"
  (cl:format cl:nil "string movementType~%int16 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomTurtleMovement-request)))
  "Returns full string definition for message of type 'CustomTurtleMovement-request"
  (cl:format cl:nil "string movementType~%int16 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomTurtleMovement-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'movementType))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomTurtleMovement-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomTurtleMovement-request
    (cl:cons ':movementType (movementType msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude CustomTurtleMovement-response.msg.html

(cl:defclass <CustomTurtleMovement-response> (roslisp-msg-protocol:ros-message)
  ((movementType
    :reader movementType
    :initarg :movementType
    :type cl:string
    :initform ""))
)

(cl:defclass CustomTurtleMovement-response (<CustomTurtleMovement-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomTurtleMovement-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomTurtleMovement-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hw_3-srv:<CustomTurtleMovement-response> is deprecated: use hw_3-srv:CustomTurtleMovement-response instead.")))

(cl:ensure-generic-function 'movementType-val :lambda-list '(m))
(cl:defmethod movementType-val ((m <CustomTurtleMovement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hw_3-srv:movementType-val is deprecated.  Use hw_3-srv:movementType instead.")
  (movementType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomTurtleMovement-response>) ostream)
  "Serializes a message object of type '<CustomTurtleMovement-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'movementType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'movementType))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomTurtleMovement-response>) istream)
  "Deserializes a message object of type '<CustomTurtleMovement-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'movementType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'movementType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomTurtleMovement-response>)))
  "Returns string type for a service object of type '<CustomTurtleMovement-response>"
  "hw_3/CustomTurtleMovementResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomTurtleMovement-response)))
  "Returns string type for a service object of type 'CustomTurtleMovement-response"
  "hw_3/CustomTurtleMovementResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomTurtleMovement-response>)))
  "Returns md5sum for a message object of type '<CustomTurtleMovement-response>"
  "f0f2a6af04cef065d9ec88019ed73d34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomTurtleMovement-response)))
  "Returns md5sum for a message object of type 'CustomTurtleMovement-response"
  "f0f2a6af04cef065d9ec88019ed73d34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomTurtleMovement-response>)))
  "Returns full string definition for message of type '<CustomTurtleMovement-response>"
  (cl:format cl:nil "string movementType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomTurtleMovement-response)))
  "Returns full string definition for message of type 'CustomTurtleMovement-response"
  (cl:format cl:nil "string movementType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomTurtleMovement-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'movementType))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomTurtleMovement-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomTurtleMovement-response
    (cl:cons ':movementType (movementType msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CustomTurtleMovement)))
  'CustomTurtleMovement-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CustomTurtleMovement)))
  'CustomTurtleMovement-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomTurtleMovement)))
  "Returns string type for a service object of type '<CustomTurtleMovement>"
  "hw_3/CustomTurtleMovement")