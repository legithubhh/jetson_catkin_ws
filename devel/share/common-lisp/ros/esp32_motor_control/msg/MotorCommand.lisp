; Auto-generated. Do not edit!


(cl:in-package esp32_motor_control-msg)


;//! \htmlinclude MotorCommand.msg.html

(cl:defclass <MotorCommand> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (steps
    :reader steps
    :initarg :steps
    :type cl:integer
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:boolean
    :initform cl:nil)
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MotorCommand (<MotorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name esp32_motor_control-msg:<MotorCommand> is deprecated: use esp32_motor_control-msg:MotorCommand instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esp32_motor_control-msg:speed-val is deprecated.  Use esp32_motor_control-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steps-val :lambda-list '(m))
(cl:defmethod steps-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esp32_motor_control-msg:steps-val is deprecated.  Use esp32_motor_control-msg:steps instead.")
  (steps m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esp32_motor_control-msg:direction-val is deprecated.  Use esp32_motor_control-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esp32_motor_control-msg:enable-val is deprecated.  Use esp32_motor_control-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommand>) ostream)
  "Serializes a message object of type '<MotorCommand>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direction) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommand>) istream)
  "Deserializes a message object of type '<MotorCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'direction) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommand>)))
  "Returns string type for a message object of type '<MotorCommand>"
  "esp32_motor_control/MotorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommand)))
  "Returns string type for a message object of type 'MotorCommand"
  "esp32_motor_control/MotorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommand>)))
  "Returns md5sum for a message object of type '<MotorCommand>"
  "940b5105a71761b8262aeb4d75bf0d20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommand)))
  "Returns md5sum for a message object of type 'MotorCommand"
  "940b5105a71761b8262aeb4d75bf0d20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommand>)))
  "Returns full string definition for message of type '<MotorCommand>"
  (cl:format cl:nil "int32 speed~%int32 steps~%bool direction~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommand)))
  "Returns full string definition for message of type 'MotorCommand"
  (cl:format cl:nil "int32 speed~%int32 steps~%bool direction~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommand>))
  (cl:+ 0
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommand
    (cl:cons ':speed (speed msg))
    (cl:cons ':steps (steps msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':enable (enable msg))
))
