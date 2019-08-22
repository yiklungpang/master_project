; Auto-generated. Do not edit!


(cl:in-package affordance_experiment-srv)


;//! \htmlinclude RecordVisuals-request.msg.html

(cl:defclass <RecordVisuals-request> (roslisp-msg-protocol:ros-message)
  ((object_list
    :reader object_list
    :initarg :object_list
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RecordVisuals-request (<RecordVisuals-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecordVisuals-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecordVisuals-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name affordance_experiment-srv:<RecordVisuals-request> is deprecated: use affordance_experiment-srv:RecordVisuals-request instead.")))

(cl:ensure-generic-function 'object_list-val :lambda-list '(m))
(cl:defmethod object_list-val ((m <RecordVisuals-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:object_list-val is deprecated.  Use affordance_experiment-srv:object_list instead.")
  (object_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecordVisuals-request>) ostream)
  "Serializes a message object of type '<RecordVisuals-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'object_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecordVisuals-request>) istream)
  "Deserializes a message object of type '<RecordVisuals-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecordVisuals-request>)))
  "Returns string type for a service object of type '<RecordVisuals-request>"
  "affordance_experiment/RecordVisualsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordVisuals-request)))
  "Returns string type for a service object of type 'RecordVisuals-request"
  "affordance_experiment/RecordVisualsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecordVisuals-request>)))
  "Returns md5sum for a message object of type '<RecordVisuals-request>"
  "c4ae6a3a033f02c4bebe23d459ae79eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordVisuals-request)))
  "Returns md5sum for a message object of type 'RecordVisuals-request"
  "c4ae6a3a033f02c4bebe23d459ae79eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordVisuals-request>)))
  "Returns full string definition for message of type '<RecordVisuals-request>"
  (cl:format cl:nil "string[] object_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordVisuals-request)))
  "Returns full string definition for message of type 'RecordVisuals-request"
  (cl:format cl:nil "string[] object_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordVisuals-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordVisuals-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordVisuals-request
    (cl:cons ':object_list (object_list msg))
))
;//! \htmlinclude RecordVisuals-response.msg.html

(cl:defclass <RecordVisuals-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass RecordVisuals-response (<RecordVisuals-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecordVisuals-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecordVisuals-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name affordance_experiment-srv:<RecordVisuals-response> is deprecated: use affordance_experiment-srv:RecordVisuals-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <RecordVisuals-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:status-val is deprecated.  Use affordance_experiment-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecordVisuals-response>) ostream)
  "Serializes a message object of type '<RecordVisuals-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecordVisuals-response>) istream)
  "Deserializes a message object of type '<RecordVisuals-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecordVisuals-response>)))
  "Returns string type for a service object of type '<RecordVisuals-response>"
  "affordance_experiment/RecordVisualsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordVisuals-response)))
  "Returns string type for a service object of type 'RecordVisuals-response"
  "affordance_experiment/RecordVisualsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecordVisuals-response>)))
  "Returns md5sum for a message object of type '<RecordVisuals-response>"
  "c4ae6a3a033f02c4bebe23d459ae79eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordVisuals-response)))
  "Returns md5sum for a message object of type 'RecordVisuals-response"
  "c4ae6a3a033f02c4bebe23d459ae79eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordVisuals-response>)))
  "Returns full string definition for message of type '<RecordVisuals-response>"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordVisuals-response)))
  "Returns full string definition for message of type 'RecordVisuals-response"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordVisuals-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordVisuals-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordVisuals-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RecordVisuals)))
  'RecordVisuals-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RecordVisuals)))
  'RecordVisuals-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordVisuals)))
  "Returns string type for a service object of type '<RecordVisuals>"
  "affordance_experiment/RecordVisuals")