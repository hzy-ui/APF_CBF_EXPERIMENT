;; Auto-generated. Do not edit!


(when (boundp 'uav_planning::Barrier_info)
  (if (not (find-package "UAV_PLANNING"))
    (make-package "UAV_PLANNING"))
  (shadow 'Barrier_info (find-package "UAV_PLANNING")))
(unless (find-package "UAV_PLANNING::BARRIER_INFO")
  (make-package "UAV_PLANNING::BARRIER_INFO"))

(in-package "ROS")
;;//! \htmlinclude Barrier_info.msg.html


(defclass uav_planning::Barrier_info
  :super ros::object
  :slots (_t _h _gamma _b _u1 _u2 _x _y _b_t ))

(defmethod uav_planning::Barrier_info
  (:init
   (&key
    ((:t __t) 0.0)
    ((:h __h) 0.0)
    ((:gamma __gamma) 0.0)
    ((:b __b) 0.0)
    ((:u1 __u1) 0.0)
    ((:u2 __u2) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:b_t __b_t) 0.0)
    )
   (send-super :init)
   (setq _t (float __t))
   (setq _h (float __h))
   (setq _gamma (float __gamma))
   (setq _b (float __b))
   (setq _u1 (float __u1))
   (setq _u2 (float __u2))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _b_t (float __b_t))
   self)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:gamma
   (&optional __gamma)
   (if __gamma (setq _gamma __gamma)) _gamma)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:u2
   (&optional __u2)
   (if __u2 (setq _u2 __u2)) _u2)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:b_t
   (&optional __b_t)
   (if __b_t (setq _b_t __b_t)) _b_t)
  (:serialization-length
   ()
   (+
    ;; float32 _t
    4
    ;; float32 _h
    4
    ;; float32 _gamma
    4
    ;; float32 _b
    4
    ;; float32 _u1
    4
    ;; float32 _u2
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _b_t
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _t
       (sys::poke _t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _h
       (sys::poke _h (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gamma
       (sys::poke _gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b
       (sys::poke _b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _u1
       (sys::poke _u1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _u2
       (sys::poke _u2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_t
       (sys::poke _b_t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _t
     (setq _t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _h
     (setq _h (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gamma
     (setq _gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b
     (setq _b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _u1
     (setq _u1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _u2
     (setq _u2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_t
     (setq _b_t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get uav_planning::Barrier_info :md5sum-) "1a5469db9dded0c3a8fee01955f5cff1")
(setf (get uav_planning::Barrier_info :datatype-) "uav_planning/Barrier_info")
(setf (get uav_planning::Barrier_info :definition-)
      "float32 t
float32 h
float32 gamma
float32 b
float32 u1
float32 u2
float32 x
float32 y
float32 b_t
")



(provide :uav_planning/Barrier_info "1a5469db9dded0c3a8fee01955f5cff1")


