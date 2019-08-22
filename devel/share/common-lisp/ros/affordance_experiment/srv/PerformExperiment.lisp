; Auto-generated. Do not edit!


(cl:in-package affordance_experiment-srv)


;//! \htmlinclude PerformExperiment-request.msg.html

(cl:defclass <PerformExperiment-request> (roslisp-msg-protocol:ros-message)
  ((tool_name
    :reader tool_name
    :initarg :tool_name
    :type cl:string
    :initform "")
   (object_list
    :reader object_list
    :initarg :object_list
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (repeat_no
    :reader repeat_no
    :initarg :repeat_no
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PerformExperiment-request (<PerformExperiment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformExperiment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformExperiment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name affordance_experiment-srv:<PerformExperiment-request> is deprecated: use affordance_experiment-srv:PerformExperiment-request instead.")))

(cl:ensure-generic-function 'tool_name-val :lambda-list '(m))
(cl:defmethod tool_name-val ((m <PerformExperiment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:tool_name-val is deprecated.  Use affordance_experiment-srv:tool_name instead.")
  (tool_name m))

(cl:ensure-generic-function 'object_list-val :lambda-list '(m))
(cl:defmethod object_list-val ((m <PerformExperiment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:object_list-val is deprecated.  Use affordance_experiment-srv:object_list instead.")
  (object_list m))

(cl:ensure-generic-function 'repeat_no-val :lambda-list '(m))
(cl:defmethod repeat_no-val ((m <PerformExperiment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:repeat_no-val is deprecated.  Use affordance_experiment-srv:repeat_no instead.")
  (repeat_no m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformExperiment-request>) ostream)
  "Serializes a message object of type '<PerformExperiment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tool_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tool_name))
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
  (cl:let* ((signed (cl:slot-value msg 'repeat_no)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformExperiment-request>) istream)
  "Deserializes a message object of type '<PerformExperiment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tool_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tool_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'repeat_no) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformExperiment-request>)))
  "Returns string type for a service object of type '<PerformExperiment-request>"
  "affordance_experiment/PerformExperimentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformExperiment-request)))
  "Returns string type for a service object of type 'PerformExperiment-request"
  "affordance_experiment/PerformExperimentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformExperiment-request>)))
  "Returns md5sum for a message object of type '<PerformExperiment-request>"
  "6f5d2f09f803b7c0cfced3acb6c92077")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformExperiment-request)))
  "Returns md5sum for a message object of type 'PerformExperiment-request"
  "6f5d2f09f803b7c0cfced3acb6c92077")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformExperiment-request>)))
  "Returns full string definition for message of type '<PerformExperiment-request>"
  (cl:format cl:nil "string tool_name~%string[] object_list~%int8 repeat_no~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformExperiment-request)))
  "Returns full string definition for message of type 'PerformExperiment-request"
  (cl:format cl:nil "string tool_name~%string[] object_list~%int8 repeat_no~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformExperiment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tool_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformExperiment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformExperiment-request
    (cl:cons ':tool_name (tool_name msg))
    (cl:cons ':object_list (object_list msg))
    (cl:cons ':repeat_no (repeat_no msg))
))
;//! \htmlinclude PerformExperiment-response.msg.html

(cl:defclass <PerformExperiment-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass PerformExperiment-response (<PerformExperiment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformExperiment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformExperiment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name affordance_experiment-srv:<PerformExperiment-response> is deprecated: use affordance_experiment-srv:PerformExperiment-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <PerformExperiment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordance_experiment-srv:status-val is deprecated.  Use affordance_experiment-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformExperiment-response>) ostream)
  "Serializes a message object of type '<PerformExperiment-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformExperiment-response>) istream)
  "Deserializes a message object of type '<PerformExperiment-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformExperiment-response>)))
  "Returns string type for a service object of type '<PerformExperiment-response>"
  "affordance_experiment/PerformExperimentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformExperiment-response)))
  "Returns string type for a service object of type 'PerformExperiment-response"
  "affordance_experiment/PerformExperimentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformExperiment-response>)))
  "Returns md5sum for a message object of type '<PerformExperiment-response>"
  "6f5d2f09f803b7c0cfced3acb6c92077")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformExperiment-response)))
  "Returns md5sum for a message object of type 'PerformExperiment-response"
  "6f5d2f09f803b7c0cfced3acb6c92077")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformExperiment-response>)))
  "Returns full string definition for message of type '<PerformExperiment-response>"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformExperiment-response)))
  "Returns full string definition for message of type 'PerformExperiment-response"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformExperiment-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformExperiment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformExperiment-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PerformExperiment)))
  'PerformExperiment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PerformExperiment)))
  'PerformExperiment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformExperiment)))
  "Returns string type for a service object of type '<PerformExperiment>"
  "affordance_experiment/PerformExperiment")