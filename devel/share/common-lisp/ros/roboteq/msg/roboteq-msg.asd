
(cl:in-package :asdf)

(defsystem "roboteq-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "roboteq_msg" :depends-on ("_package_roboteq_msg"))
    (:file "_package_roboteq_msg" :depends-on ("_package"))
  ))