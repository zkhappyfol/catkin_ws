;; Auto-generated. Do not edit!


(when (boundp 'my_first_robot::DetectedObject)
  (if (not (find-package "MY_FIRST_ROBOT"))
    (make-package "MY_FIRST_ROBOT"))
  (shadow 'DetectedObject (find-package "MY_FIRST_ROBOT")))
(unless (find-package "MY_FIRST_ROBOT::DETECTEDOBJECT")
  (make-package "MY_FIRST_ROBOT::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html


(defclass my_first_robot::DetectedObject
  :super ros::object
  :slots (_color _pixel_x _pixel_y _area ))

(defmethod my_first_robot::DetectedObject
  (:init
   (&key
    ((:color __color) "")
    ((:pixel_x __pixel_x) 0)
    ((:pixel_y __pixel_y) 0)
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _color (string __color))
   (setq _pixel_x (round __pixel_x))
   (setq _pixel_y (round __pixel_y))
   (setq _area (float __area))
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:pixel_x
   (&optional __pixel_x)
   (if __pixel_x (setq _pixel_x __pixel_x)) _pixel_x)
  (:pixel_y
   (&optional __pixel_y)
   (if __pixel_y (setq _pixel_y __pixel_y)) _pixel_y)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; string _color
    4 (length _color)
    ;; int32 _pixel_x
    4
    ;; int32 _pixel_y
    4
    ;; float32 _area
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; int32 _pixel_x
       (write-long _pixel_x s)
     ;; int32 _pixel_y
       (write-long _pixel_y s)
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _pixel_x
     (setq _pixel_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pixel_y
     (setq _pixel_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_first_robot::DetectedObject :md5sum-) "e11067cb68b62a3f0e378a2a08e38707")
(setf (get my_first_robot::DetectedObject :datatype-) "my_first_robot/DetectedObject")
(setf (get my_first_robot::DetectedObject :definition-)
      "string color
int32 pixel_x
int32 pixel_y
float32 area
")



(provide :my_first_robot/DetectedObject "e11067cb68b62a3f0e378a2a08e38707")


