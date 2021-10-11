
(cl:in-package :asdf)

(defsystem "lab1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MapperMessage" :depends-on ("_package_MapperMessage"))
    (:file "_package_MapperMessage" :depends-on ("_package"))
    (:file "MapperMessage" :depends-on ("_package_MapperMessage"))
    (:file "_package_MapperMessage" :depends-on ("_package"))
  ))