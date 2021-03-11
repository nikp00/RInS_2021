;; Auto-generated. Do not edit!


(when (boundp 'hw_3::CustomTurtleMovement)
  (if (not (find-package "HW_3"))
    (make-package "HW_3"))
  (shadow 'CustomTurtleMovement (find-package "HW_3")))
(unless (find-package "HW_3::CUSTOMTURTLEMOVEMENT")
  (make-package "HW_3::CUSTOMTURTLEMOVEMENT"))
(unless (find-package "HW_3::CUSTOMTURTLEMOVEMENTREQUEST")
  (make-package "HW_3::CUSTOMTURTLEMOVEMENTREQUEST"))
(unless (find-package "HW_3::CUSTOMTURTLEMOVEMENTRESPONSE")
  (make-package "HW_3::CUSTOMTURTLEMOVEMENTRESPONSE"))

(in-package "ROS")





(defclass hw_3::CustomTurtleMovementRequest
  :super ros::object
  :slots (_movementType _duration ))

(defmethod hw_3::CustomTurtleMovementRequest
  (:init
   (&key
    ((:movementType __movementType) "")
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _movementType (string __movementType))
   (setq _duration (round __duration))
   self)
  (:movementType
   (&optional __movementType)
   (if __movementType (setq _movementType __movementType)) _movementType)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; string _movementType
    4 (length _movementType)
    ;; int16 _duration
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _movementType
       (write-long (length _movementType) s) (princ _movementType s)
     ;; int16 _duration
       (write-word _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _movementType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _movementType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _duration
     (setq _duration (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass hw_3::CustomTurtleMovementResponse
  :super ros::object
  :slots (_movementType ))

(defmethod hw_3::CustomTurtleMovementResponse
  (:init
   (&key
    ((:movementType __movementType) "")
    )
   (send-super :init)
   (setq _movementType (string __movementType))
   self)
  (:movementType
   (&optional __movementType)
   (if __movementType (setq _movementType __movementType)) _movementType)
  (:serialization-length
   ()
   (+
    ;; string _movementType
    4 (length _movementType)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _movementType
       (write-long (length _movementType) s) (princ _movementType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _movementType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _movementType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hw_3::CustomTurtleMovement
  :super ros::object
  :slots ())

(setf (get hw_3::CustomTurtleMovement :md5sum-) "f0f2a6af04cef065d9ec88019ed73d34")
(setf (get hw_3::CustomTurtleMovement :datatype-) "hw_3/CustomTurtleMovement")
(setf (get hw_3::CustomTurtleMovement :request) hw_3::CustomTurtleMovementRequest)
(setf (get hw_3::CustomTurtleMovement :response) hw_3::CustomTurtleMovementResponse)

(defmethod hw_3::CustomTurtleMovementRequest
  (:response () (instance hw_3::CustomTurtleMovementResponse :init)))

(setf (get hw_3::CustomTurtleMovementRequest :md5sum-) "f0f2a6af04cef065d9ec88019ed73d34")
(setf (get hw_3::CustomTurtleMovementRequest :datatype-) "hw_3/CustomTurtleMovementRequest")
(setf (get hw_3::CustomTurtleMovementRequest :definition-)
      "string movementType
int16 duration
---
string movementType
")

(setf (get hw_3::CustomTurtleMovementResponse :md5sum-) "f0f2a6af04cef065d9ec88019ed73d34")
(setf (get hw_3::CustomTurtleMovementResponse :datatype-) "hw_3/CustomTurtleMovementResponse")
(setf (get hw_3::CustomTurtleMovementResponse :definition-)
      "string movementType
int16 duration
---
string movementType
")



(provide :hw_3/CustomTurtleMovement "f0f2a6af04cef065d9ec88019ed73d34")


