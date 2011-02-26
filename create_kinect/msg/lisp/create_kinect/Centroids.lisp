; Auto-generated. Do not edit!


(in-package create_kinect-msg)


;//! \htmlinclude Centroids.msg.html

(defclass <Centroids> (ros-message)
  ((centlx
    :reader centlx-val
    :initarg :centlx
    :type float
    :initform 0.0)
   (centlz
    :reader centlz-val
    :initarg :centlz
    :type float
    :initform 0.0)
   (centrx
    :reader centrx-val
    :initarg :centrx
    :type float
    :initform 0.0)
   (centrz
    :reader centrz-val
    :initarg :centrz
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Centroids>) ostream)
  "Serializes a message object of type '<Centroids>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'centlx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'centlz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'centrx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'centrz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Centroids>) istream)
  "Deserializes a message object of type '<Centroids>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'centlx) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'centlz) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'centrx) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'centrz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Centroids>)))
  "Returns string type for a message object of type '<Centroids>"
  "create_kinect/Centroids")
(defmethod md5sum ((type (eql '<Centroids>)))
  "Returns md5sum for a message object of type '<Centroids>"
  "598e1fd1299b9822dcea6db9a9d8636c")
(defmethod message-definition ((type (eql '<Centroids>)))
  "Returns full string definition for message of type '<Centroids>"
  (format nil "float32 centlx~%float32 centlz~%float32 centrx~%float32 centrz~%~%"))
(defmethod serialization-length ((msg <Centroids>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <Centroids>))
  "Converts a ROS message object to a list"
  (list '<Centroids>
    (cons ':centlx (centlx-val msg))
    (cons ':centlz (centlz-val msg))
    (cons ':centrx (centrx-val msg))
    (cons ':centrz (centrz-val msg))
))
