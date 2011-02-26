; Auto-generated. Do not edit!


(in-package create_kinect-msg)


;//! \htmlinclude Mode.msg.html

(defclass <Mode> (ros-message)
  ((mode
    :reader mode-val
    :initarg :mode
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Mode>) ostream)
  "Serializes a message object of type '<Mode>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'mode)) ostream)
)
(defmethod deserialize ((msg <Mode>) istream)
  "Deserializes a message object of type '<Mode>"
  (setf (ldb (byte 8 0) (slot-value msg 'mode)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Mode>)))
  "Returns string type for a message object of type '<Mode>"
  "create_kinect/Mode")
(defmethod md5sum ((type (eql '<Mode>)))
  "Returns md5sum for a message object of type '<Mode>"
  "89b81386720be1cd0ce7a3953fcd1b19")
(defmethod message-definition ((type (eql '<Mode>)))
  "Returns full string definition for message of type '<Mode>"
  (format nil "uint8 mode~%~%"))
(defmethod serialization-length ((msg <Mode>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Mode>))
  "Converts a ROS message object to a list"
  (list '<Mode>
    (cons ':mode (mode-val msg))
))
