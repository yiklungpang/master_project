;; Auto-generated. Do not edit!


(when (boundp 'affordance_experiment::PerformExperiment)
  (if (not (find-package "AFFORDANCE_EXPERIMENT"))
    (make-package "AFFORDANCE_EXPERIMENT"))
  (shadow 'PerformExperiment (find-package "AFFORDANCE_EXPERIMENT")))
(unless (find-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENT")
  (make-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENT"))
(unless (find-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENTREQUEST")
  (make-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENTREQUEST"))
(unless (find-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENTRESPONSE")
  (make-package "AFFORDANCE_EXPERIMENT::PERFORMEXPERIMENTRESPONSE"))

(in-package "ROS")





(defclass affordance_experiment::PerformExperimentRequest
  :super ros::object
  :slots (_tool_name _object_list _repeat_no ))

(defmethod affordance_experiment::PerformExperimentRequest
  (:init
   (&key
    ((:tool_name __tool_name) "")
    ((:object_list __object_list) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:repeat_no __repeat_no) 0)
    )
   (send-super :init)
   (setq _tool_name (string __tool_name))
   (setq _object_list __object_list)
   (setq _repeat_no (round __repeat_no))
   self)
  (:tool_name
   (&optional __tool_name)
   (if __tool_name (setq _tool_name __tool_name)) _tool_name)
  (:object_list
   (&optional __object_list)
   (if __object_list (setq _object_list __object_list)) _object_list)
  (:repeat_no
   (&optional __repeat_no)
   (if __repeat_no (setq _repeat_no __repeat_no)) _repeat_no)
  (:serialization-length
   ()
   (+
    ;; string _tool_name
    4 (length _tool_name)
    ;; string[] _object_list
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _object_list)) 4
    ;; int8 _repeat_no
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _tool_name
       (write-long (length _tool_name) s) (princ _tool_name s)
     ;; string[] _object_list
     (write-long (length _object_list) s)
     (dolist (elem _object_list)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int8 _repeat_no
       (write-byte _repeat_no s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _tool_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tool_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _object_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_list (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _object_list i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int8 _repeat_no
     (setq _repeat_no (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _repeat_no 127) (setq _repeat_no (- _repeat_no 256)))
   ;;
   self)
  )

(defclass affordance_experiment::PerformExperimentResponse
  :super ros::object
  :slots (_status ))

(defmethod affordance_experiment::PerformExperimentResponse
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

(defclass affordance_experiment::PerformExperiment
  :super ros::object
  :slots ())

(setf (get affordance_experiment::PerformExperiment :md5sum-) "6f5d2f09f803b7c0cfced3acb6c92077")
(setf (get affordance_experiment::PerformExperiment :datatype-) "affordance_experiment/PerformExperiment")
(setf (get affordance_experiment::PerformExperiment :request) affordance_experiment::PerformExperimentRequest)
(setf (get affordance_experiment::PerformExperiment :response) affordance_experiment::PerformExperimentResponse)

(defmethod affordance_experiment::PerformExperimentRequest
  (:response () (instance affordance_experiment::PerformExperimentResponse :init)))

(setf (get affordance_experiment::PerformExperimentRequest :md5sum-) "6f5d2f09f803b7c0cfced3acb6c92077")
(setf (get affordance_experiment::PerformExperimentRequest :datatype-) "affordance_experiment/PerformExperimentRequest")
(setf (get affordance_experiment::PerformExperimentRequest :definition-)
      "string tool_name
string[] object_list
int8 repeat_no
---
string status
")

(setf (get affordance_experiment::PerformExperimentResponse :md5sum-) "6f5d2f09f803b7c0cfced3acb6c92077")
(setf (get affordance_experiment::PerformExperimentResponse :datatype-) "affordance_experiment/PerformExperimentResponse")
(setf (get affordance_experiment::PerformExperimentResponse :definition-)
      "string tool_name
string[] object_list
int8 repeat_no
---
string status
")



(provide :affordance_experiment/PerformExperiment "6f5d2f09f803b7c0cfced3acb6c92077")


