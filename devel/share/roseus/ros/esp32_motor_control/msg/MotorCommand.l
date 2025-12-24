;; Auto-generated. Do not edit!


(when (boundp 'esp32_motor_control::MotorCommand)
  (if (not (find-package "ESP32_MOTOR_CONTROL"))
    (make-package "ESP32_MOTOR_CONTROL"))
  (shadow 'MotorCommand (find-package "ESP32_MOTOR_CONTROL")))
(unless (find-package "ESP32_MOTOR_CONTROL::MOTORCOMMAND")
  (make-package "ESP32_MOTOR_CONTROL::MOTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MotorCommand.msg.html


(defclass esp32_motor_control::MotorCommand
  :super ros::object
  :slots (_speed _steps _direction _enable ))

(defmethod esp32_motor_control::MotorCommand
  (:init
   (&key
    ((:speed __speed) 0)
    ((:steps __steps) 0)
    ((:direction __direction) nil)
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _speed (round __speed))
   (setq _steps (round __steps))
   (setq _direction __direction)
   (setq _enable __enable)
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steps
   (&optional __steps)
   (if __steps (setq _steps __steps)) _steps)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; int32 _speed
    4
    ;; int32 _steps
    4
    ;; bool _direction
    1
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _speed
       (write-long _speed s)
     ;; int32 _steps
       (write-long _steps s)
     ;; bool _direction
       (if _direction (write-byte -1 s) (write-byte 0 s))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _steps
     (setq _steps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _direction
     (setq _direction (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get esp32_motor_control::MotorCommand :md5sum-) "940b5105a71761b8262aeb4d75bf0d20")
(setf (get esp32_motor_control::MotorCommand :datatype-) "esp32_motor_control/MotorCommand")
(setf (get esp32_motor_control::MotorCommand :definition-)
      "int32 speed
int32 steps
bool direction
bool enable

")



(provide :esp32_motor_control/MotorCommand "940b5105a71761b8262aeb4d75bf0d20")


