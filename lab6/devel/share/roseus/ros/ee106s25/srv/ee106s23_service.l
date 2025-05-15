;; Auto-generated. Do not edit!


(when (boundp 'ee106s25::ee106s23_service)
  (if (not (find-package "EE106S25"))
    (make-package "EE106S25"))
  (shadow 'ee106s23_service (find-package "EE106S25")))
(unless (find-package "EE106S25::EE106S23_SERVICE")
  (make-package "EE106S25::EE106S23_SERVICE"))
(unless (find-package "EE106S25::EE106S23_SERVICEREQUEST")
  (make-package "EE106S25::EE106S23_SERVICEREQUEST"))
(unless (find-package "EE106S25::EE106S23_SERVICERESPONSE")
  (make-package "EE106S25::EE106S23_SERVICERESPONSE"))

(in-package "ROS")





(defclass ee106s25::ee106s23_serviceRequest
  :super ros::object
  :slots (_request_msg ))

(defmethod ee106s25::ee106s23_serviceRequest
  (:init
   (&key
    ((:request_msg __request_msg) "")
    )
   (send-super :init)
   (setq _request_msg (string __request_msg))
   self)
  (:request_msg
   (&optional __request_msg)
   (if __request_msg (setq _request_msg __request_msg)) _request_msg)
  (:serialization-length
   ()
   (+
    ;; string _request_msg
    4 (length _request_msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _request_msg
       (write-long (length _request_msg) s) (princ _request_msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _request_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ee106s25::ee106s23_serviceResponse
  :super ros::object
  :slots (_response_msg ))

(defmethod ee106s25::ee106s23_serviceResponse
  (:init
   (&key
    ((:response_msg __response_msg) "")
    )
   (send-super :init)
   (setq _response_msg (string __response_msg))
   self)
  (:response_msg
   (&optional __response_msg)
   (if __response_msg (setq _response_msg __response_msg)) _response_msg)
  (:serialization-length
   ()
   (+
    ;; string _response_msg
    4 (length _response_msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response_msg
       (write-long (length _response_msg) s) (princ _response_msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ee106s25::ee106s23_service
  :super ros::object
  :slots ())

(setf (get ee106s25::ee106s23_service :md5sum-) "bbbdbd457d086498a93ac52318048de7")
(setf (get ee106s25::ee106s23_service :datatype-) "ee106s25/ee106s23_service")
(setf (get ee106s25::ee106s23_service :request) ee106s25::ee106s23_serviceRequest)
(setf (get ee106s25::ee106s23_service :response) ee106s25::ee106s23_serviceResponse)

(defmethod ee106s25::ee106s23_serviceRequest
  (:response () (instance ee106s25::ee106s23_serviceResponse :init)))

(setf (get ee106s25::ee106s23_serviceRequest :md5sum-) "bbbdbd457d086498a93ac52318048de7")
(setf (get ee106s25::ee106s23_serviceRequest :datatype-) "ee106s25/ee106s23_serviceRequest")
(setf (get ee106s25::ee106s23_serviceRequest :definition-)
      "string request_msg
---
string response_msg
")

(setf (get ee106s25::ee106s23_serviceResponse :md5sum-) "bbbdbd457d086498a93ac52318048de7")
(setf (get ee106s25::ee106s23_serviceResponse :datatype-) "ee106s25/ee106s23_serviceResponse")
(setf (get ee106s25::ee106s23_serviceResponse :definition-)
      "string request_msg
---
string response_msg
")



(provide :ee106s25/ee106s23_service "bbbdbd457d086498a93ac52318048de7")


