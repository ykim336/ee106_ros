
(cl:in-package :asdf)

(defsystem "ee106s25-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ee106s23_service" :depends-on ("_package_ee106s23_service"))
    (:file "_package_ee106s23_service" :depends-on ("_package"))
  ))