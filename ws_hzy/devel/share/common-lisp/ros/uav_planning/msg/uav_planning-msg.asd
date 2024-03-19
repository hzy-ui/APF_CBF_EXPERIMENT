
(cl:in-package :asdf)

(defsystem "uav_planning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Barrier_info" :depends-on ("_package_Barrier_info"))
    (:file "_package_Barrier_info" :depends-on ("_package"))
  ))