
(cl:in-package :asdf)

(defsystem "affordance_experiment-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PerformExperiment" :depends-on ("_package_PerformExperiment"))
    (:file "_package_PerformExperiment" :depends-on ("_package"))
    (:file "RecordVisuals" :depends-on ("_package_RecordVisuals"))
    (:file "_package_RecordVisuals" :depends-on ("_package"))
  ))