#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import tensorflow as tf
import math

#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append("/home/chen/projects/tfobdetection/models/research/object_detection")

#cv bridge
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util



class CNN_Detector:
    def __init__(self):
        #initialize the parameters
        self.param1 = 0
        self.param2 = 0
        self.param1 = rospy.get_param("~param1", self.param1)
        self.param2 = rospy.get_param('~param2', self.param2)
        
        # model name
        MODEL_NAME = 'inference_graph'
        # Grab path to current working directory
        CWD_PATH = '/home/chen/projects/tfobdetection/models/research/object_detection'
        # Path to frozen detection graph .pb file, which contains the model that is used
        # for object detection.
        PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')
        # Path to label map file
        PATH_TO_LABELS = os.path.join(CWD_PATH,'training','labelmap.pbtxt')
        # Path to image
 #       PATH_TO_IMAGE = os.path.join(CWD_PATH,IMAGE_NAME)
        # Number of classes the object detector can identify
        NUM_CLASSES = 1
        
        # Load the label map. 
        # Label maps map indices to category names, so that when our convolution
        # network predicts `5`, we know that this corresponds to `king`.
        # Here we use internal utility functions, but anything that returns a
        # dictionary mapping integers to appropriate string labels would be fine
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        # Load the Tensorflow model into memory.
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
            self.sess = tf.Session(graph=detection_graph)
        # Define input and output tensors (i.e. data) for the object detection classifier
        # Input tensor is the image
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Output tensors are the detection boxes, scores, and classes
        # Each box represents a part of the image where a particular object was detected
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represents level of confidence for each of the objects.
        # The score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        # Number of objects detected
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        # Load image using OpenCV and
        # expand image dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value

        #boundingbox publisher
        self.pub_boundingbox = rospy.Publisher("/charge_detector/boundingbox", BoundingBox, queue_size = 1)

        #subscribe to the image
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/realsense/camera/color/image_raw",Image, self.inference_image)


    #delete function
    def __del__(self):
        # Clean up
        cv2.destroyAllWindows()


    def inference_image(self,data):

        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return 
        
        #if image captured
        image_expanded = np.expand_dims(image, axis=0)
        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: image_expanded})
        # Draw the results of the detection (aka 'visulaize the results')
        # vis_util.visualize_boxes_and_labels_on_image_array(
        #     image,
        #     np.squeeze(boxes),
        #     np.squeeze(classes).astype(np.int32),
        #     np.squeeze(scores),
        #     self.category_index,
        #     use_normalized_coordinates=True,
        #     line_thickness=8,
        #     min_score_thresh=0.80)
        if len(boxes[0]) > 0 and scores[0][0] > 0.8:
            [min_x, min_y, max_x, max_y] = boxes[0][0]
            height, width = image.shape[:2]
            # print(min_x * height, min_y * width, max_x * height, max_y * width)
            # cv2.rectangle(image, (int(min_y * width), int(min_x * height)), (int(max_y * width), int(max_x * height)), (255, 0, 0), 2)
            boundingbox = BoundingBox()
            boundingbox.header = data.header
            boundingbox.pose.position.x = int(min_y * width)
            boundingbox.pose.position.y = int(min_x * height)
            boundingbox.dimensions.x = int((max_y - min_y) * width)
            boundingbox.dimensions.y = int((max_x - min_x) * height)
            # print(boundingbox)
            self.pub_boundingbox.publish(boundingbox)
        # All the results have been drawn on image. Now display the image.
            cv2.imshow('Object detector', image)
            if cv2.waitKey(10)==ord('q'):
                return


if __name__ == "__main__":
    rospy.init_node('Charge_detector')
    ex = CNN_Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
      # Clean up
    cv2.destroyAllWindows()
