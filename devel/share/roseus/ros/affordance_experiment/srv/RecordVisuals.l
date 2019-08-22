;; Auto-generated. Do not edit!


(when (boundp 'affordance_experiment::RecordVisuals)
  (if (not (find-package "AFFORDANCE_EXPERIMENT"))
    (make-package "AFFORDANCE_EXPERIMENT"))
  (shadow 'RecordVisuals (find-package "AFFORDANCE_EXPERIMENT")))
(unless (find-package "AFFORDANCE_EXPERIMENT::RECORDVISUALS")
  (make-package "AFFORDANCE_EXPERIMENT::RECORDVISUALS"))
(unless (find-package "AFFORDANCE_EXPERIMENT::RECORDVISUALSREQUEST")
  (make-package "AFFORDANCE_EXPERIMENT::RECORDVISUALSREQUEST"))
(unless (find-package "AFFORDANCE_EXPERIMENT::RECORDVISUALSRESPONSE")
  (make-package "AFFORDANCE_EXPERIMENT::RECORDVISUALSRESPONSE"))

(in-package "ROS")





(defclass affordance_experiment::RecordVisualsRequest
  :super ros::object
  :slots (_object_list ))

(defmethod affordance_experiment::RecordVisualsRequest
  (:init
   (&key
    ((:object_list __object_list) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _object_list __object_list)
   self)
  (:object_list
   (&optional __object_list)
   (if __object_list (setq _object_list __object_list)) _object_list)
  (:serialization-length
   ()
   (+
    ;; string[] _object_list
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _object_list)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _object_list
     (write-long (length _object_list) s)
     (dolist (elem _object_list)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _object_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_list (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _object_list i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass affordance_experiment::RecordVisualsResponse
  :super ros::object
  :slots (_status ))

(defmethod affordance_experiment::RecordVisualsResponse
  (:init
   (&key
    ((:status __status) "")
    )
   (send-super :init)
   (setq _status (string __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass affordance_experiment::RecordVisuals
  :super ros::object
  :slots ())

(setf (get affordance_experiment::RecordVisuals :md5sum-) "c4ae6a3a033f02c4bebe23d459ae79eb")
(setf (get affordance_experiment::RecordVisuals :datatype-) "affordance_experiment/RecordVisuals")
(setf (get affordance_experiment::RecordVisuals :request) affordance_experiment::RecordVisualsRequest)
(setf (get affordance_experiment::RecordVisuals :response) affordance_experiment::RecordVisualsResponse)

(defmethod affordance_experiment::RecordVisualsRequest
  (:response () (instance affordance_experiment::RecordVisualsResponse :init)))

(setf (get affordance_experiment::RecordVisualsRequest :md5sum-) "c4ae6a3a033f02c4bebe23d459ae79eb")
(setf (get affordance_experiment::RecordVisualsRequest :datatype-) "affordance_experiment/RecordVisualsRequest")
(setf (get affordance_experiment::RecordVisualsRequest :definition-)
      "string[] object_list
---
string status
")

(setf (get affordance_experiment::RecordVisualsResponse :md5sum-) "c4ae6a3a033f02c4bebe23d459ae79eb")
(setf (get affordance_experiment::RecordVisualsResponse :datatype-) "affordance_experiment/RecordVisualsResponse")
(setf (get affordance_experiment::RecordVisualsResponse :definition-)
      "string[] object_list
---
string status
")



(provide :affordance_experiment/RecordVisuals "c4ae6a3a033f02c4bebe23d459ae79eb")


