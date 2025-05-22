; Auto-generated. Do not edit!


(cl:in-package ee106s25-srv)


;//! \htmlinclude ee106s23_service-request.msg.html

(cl:defclass <ee106s23_service-request> (roslisp-msg-protocol:ros-message)
  ((request_msg
    :reader request_msg
    :initarg :request_msg
    :type cl:string
    :initform ""))
)

(cl:defclass ee106s23_service-request (<ee106s23_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ee106s23_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ee106s23_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee106s25-srv:<ee106s23_service-request> is deprecated: use ee106s25-srv:ee106s23_service-request instead.")))

(cl:ensure-generic-function 'request_msg-val :lambda-list '(m))
(cl:defmethod request_msg-val ((m <ee106s23_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee106s25-srv:request_msg-val is deprecated.  Use ee106s25-srv:request_msg instead.")
  (request_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ee106s23_service-request>) ostream)
  "Serializes a message object of type '<ee106s23_service-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request_msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ee106s23_service-request>) istream)
  "Deserializes a message object of type '<ee106s23_service-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ee106s23_service-request>)))
  "Returns string type for a service object of type '<ee106s23_service-request>"
  "ee106s25/ee106s23_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ee106s23_service-request)))
  "Returns string type for a service object of type 'ee106s23_service-request"
  "ee106s25/ee106s23_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ee106s23_service-request>)))
  "Returns md5sum for a message object of type '<ee106s23_service-request>"
  "bbbdbd457d086498a93ac52318048de7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ee106s23_service-request)))
  "Returns md5sum for a message object of type 'ee106s23_service-request"
  "bbbdbd457d086498a93ac52318048de7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ee106s23_service-request>)))
  "Returns full string definition for message of type '<ee106s23_service-request>"
  (cl:format cl:nil "string request_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ee106s23_service-request)))
  "Returns full string definition for message of type 'ee106s23_service-request"
  (cl:format cl:nil "string request_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ee106s23_service-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ee106s23_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ee106s23_service-request
    (cl:cons ':request_msg (request_msg msg))
))
;//! \htmlinclude ee106s23_service-response.msg.html

(cl:defclass <ee106s23_service-response> (roslisp-msg-protocol:ros-message)
  ((response_msg
    :reader response_msg
    :initarg :response_msg
    :type cl:string
    :initform ""))
)

(cl:defclass ee106s23_service-response (<ee106s23_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ee106s23_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ee106s23_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee106s25-srv:<ee106s23_service-response> is deprecated: use ee106s25-srv:ee106s23_service-response instead.")))

(cl:ensure-generic-function 'response_msg-val :lambda-list '(m))
(cl:defmethod response_msg-val ((m <ee106s23_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee106s25-srv:response_msg-val is deprecated.  Use ee106s25-srv:response_msg instead.")
  (response_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ee106s23_service-response>) ostream)
  "Serializes a message object of type '<ee106s23_service-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response_msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ee106s23_service-response>) istream)
  "Deserializes a message object of type '<ee106s23_service-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ee106s23_service-response>)))
  "Returns string type for a service object of type '<ee106s23_service-response>"
  "ee106s25/ee106s23_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ee106s23_service-response)))
  "Returns string type for a service object of type 'ee106s23_service-response"
  "ee106s25/ee106s23_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ee106s23_service-response>)))
  "Returns md5sum for a message object of type '<ee106s23_service-response>"
  "bbbdbd457d086498a93ac52318048de7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ee106s23_service-response)))
  "Returns md5sum for a message object of type 'ee106s23_service-response"
  "bbbdbd457d086498a93ac52318048de7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ee106s23_service-response>)))
  "Returns full string definition for message of type '<ee106s23_service-response>"
  (cl:format cl:nil "string response_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ee106s23_service-response)))
  "Returns full string definition for message of type 'ee106s23_service-response"
  (cl:format cl:nil "string response_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ee106s23_service-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ee106s23_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ee106s23_service-response
    (cl:cons ':response_msg (response_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ee106s23_service)))
  'ee106s23_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ee106s23_service)))
  'ee106s23_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ee106s23_service)))
  "Returns string type for a service object of type '<ee106s23_service>"
  "ee106s25/ee106s23_service")