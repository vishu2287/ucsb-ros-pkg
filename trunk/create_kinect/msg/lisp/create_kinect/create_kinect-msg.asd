
(in-package :asdf)

(defsystem "create_kinect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Centroids" :depends-on ("_package"))
    (:file "_package_Centroids" :depends-on ("_package"))
    (:file "Mode" :depends-on ("_package"))
    (:file "_package_Mode" :depends-on ("_package"))
    ))
