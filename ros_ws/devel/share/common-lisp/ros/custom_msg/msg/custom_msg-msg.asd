
(cl:in-package :asdf)

(defsystem "custom_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "arm" :depends-on ("_package_arm"))
    (:file "_package_arm" :depends-on ("_package"))
  ))