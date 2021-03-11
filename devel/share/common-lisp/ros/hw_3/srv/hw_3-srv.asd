
(cl:in-package :asdf)

(defsystem "hw_3-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CustomTurtleMovement" :depends-on ("_package_CustomTurtleMovement"))
    (:file "_package_CustomTurtleMovement" :depends-on ("_package"))
  ))