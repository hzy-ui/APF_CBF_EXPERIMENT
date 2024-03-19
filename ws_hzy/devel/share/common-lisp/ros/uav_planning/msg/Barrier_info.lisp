; Auto-generated. Do not edit!


(cl:in-package uav_planning-msg)


;//! \htmlinclude Barrier_info.msg.html

(cl:defclass <Barrier_info> (roslisp-msg-protocol:ros-message)
  ((t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0)
   (h
    :reader h
    :initarg :h
    :type cl:float
    :initform 0.0)
   (gamma
    :reader gamma
    :initarg :gamma
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0)
   (u1
    :reader u1
    :initarg :u1
    :type cl:float
    :initform 0.0)
   (u2
    :reader u2
    :initarg :u2
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (b_t
    :reader b_t
    :initarg :b_t
    :type cl:float
    :initform 0.0))
)

(cl:defclass Barrier_info (<Barrier_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Barrier_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Barrier_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_planning-msg:<Barrier_info> is deprecated: use uav_planning-msg:Barrier_info instead.")))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:t-val is deprecated.  Use uav_planning-msg:t instead.")
  (t m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:h-val is deprecated.  Use uav_planning-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'gamma-val :lambda-list '(m))
(cl:defmethod gamma-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:gamma-val is deprecated.  Use uav_planning-msg:gamma instead.")
  (gamma m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:b-val is deprecated.  Use uav_planning-msg:b instead.")
  (b m))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:u1-val is deprecated.  Use uav_planning-msg:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'u2-val :lambda-list '(m))
(cl:defmethod u2-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:u2-val is deprecated.  Use uav_planning-msg:u2 instead.")
  (u2 m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:x-val is deprecated.  Use uav_planning-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:y-val is deprecated.  Use uav_planning-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'b_t-val :lambda-list '(m))
(cl:defmethod b_t-val ((m <Barrier_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_planning-msg:b_t-val is deprecated.  Use uav_planning-msg:b_t instead.")
  (b_t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Barrier_info>) ostream)
  "Serializes a message object of type '<Barrier_info>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'h))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Barrier_info>) istream)
  "Deserializes a message object of type '<Barrier_info>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'h) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gamma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_t) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Barrier_info>)))
  "Returns string type for a message object of type '<Barrier_info>"
  "uav_planning/Barrier_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Barrier_info)))
  "Returns string type for a message object of type 'Barrier_info"
  "uav_planning/Barrier_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Barrier_info>)))
  "Returns md5sum for a message object of type '<Barrier_info>"
  "1a5469db9dded0c3a8fee01955f5cff1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Barrier_info)))
  "Returns md5sum for a message object of type 'Barrier_info"
  "1a5469db9dded0c3a8fee01955f5cff1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Barrier_info>)))
  "Returns full string definition for message of type '<Barrier_info>"
  (cl:format cl:nil "float32 t~%float32 h~%float32 gamma~%float32 b~%float32 u1~%float32 u2~%float32 x~%float32 y~%float32 b_t~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Barrier_info)))
  "Returns full string definition for message of type 'Barrier_info"
  (cl:format cl:nil "float32 t~%float32 h~%float32 gamma~%float32 b~%float32 u1~%float32 u2~%float32 x~%float32 y~%float32 b_t~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Barrier_info>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Barrier_info>))
  "Converts a ROS message object to a list"
  (cl:list 'Barrier_info
    (cl:cons ':t (t msg))
    (cl:cons ':h (h msg))
    (cl:cons ':gamma (gamma msg))
    (cl:cons ':b (b msg))
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':u2 (u2 msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':b_t (b_t msg))
))
