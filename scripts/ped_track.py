#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('frozone')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError

# Tracking and Prediction headers
import os
import time
import datetime
import torch
from torch.autograd import Variable
import torchvision.transforms as transforms
import numpy as np
import tensorflow as tf
from PIL import Image
import warnings
import sys
from utils.yolo.yolo_model import *
from utils.yolo.utils import *
from utils.yolo.datasets import *
from utils.generate_features import *
from utils.deep_sort import nn_matching
from utils.deep_sort.detection import Detection
from utils.deep_sort.tracker import Tracker
from utils.application_util import preprocessing
from utils.application_util import visualization
import scipy.misc
import imageio

np.set_printoptions(threshold=sys.maxsize)

# Locations of the trained model
detector_cfg_loc = "/home/asathyam/code/RealTimeTracking/resources/yolo-coco/yolov3.cfg"
detector_weights_loc = "/home/asathyam/code/RealTimeTracking/resources/yolo-coco/yolov3.weights"
detector_classes_loc = "/home/asathyam/code/RealTimeTracking/resources/yolo-coco/coco.names"
box_encoder_loc = "/home/asathyam/code/RealTimeTracking/resources/mars-small128.pb"
save_loc = "."

def rgb2gray(rgb):

    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2] # Doesn't opencv use BGR?
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b

    # return gray
    return gray

def create_detections(detection_mat, frame_idx, min_height=0):
    """Create detections for given frame index from the raw detection matrix.

    Parameters
    ----------
    detection_mat : ndarray
        Matrix of detections. The first 10 columns of the detection matrix are
        in the standard MOTChallenge detection format. In the remaining columns
        store the feature vector associated with each detection.
    frame_idx : int
        The frame index.
    min_height : Optional[int]
        A minimum detection bounding box height. Detections that are smaller
        than this value are disregarded.

    Returns
    -------
    List[tracker.Detection]
        Returns detection responses at given frame index.

    """
    det_lst = [lst for lst in detection_mat if lst[0] == frame_idx]

    detection_list = []
    for row in det_lst:
        bbox, confidence, agent_cls, feature = row[2:6], row[6], row[10], row[11:]
        if bbox[3] < min_height:
            continue
        detection_list.append(Detection(bbox, confidence, agent_cls, feature))
    return detection_list


class Memory(object):

    def __init__(self, detargs, trackargs, cuda=False, img_size=416, display=False):

        self.reset_num = 0
        self.frame_id = 0
        self.raw_det = []
        self.feature_det = []
        self.tracking = []
        self.track_dic = {}
        self.pred_means = None
        self.pred_covariances = None

        self.img_paths = []
        self.loc = save_loc

        self.device = torch.device("cuda" if cuda and torch.cuda.is_available() else "cpu")
        self.Tensor = torch.cuda.FloatTensor if cuda and torch.cuda.is_available() else torch.FloatTensor
        self.img_size = img_size
        self.display = display

        self.detargs = detargs
        self.detector = Darknet(detector_cfg_loc).to(self.device)
        self.detector.load_darknet_weights(detector_weights_loc)
        self.detector.eval()
        # self.classes = load_classes(detector_classes_loc)

        self.encoder = create_box_encoder(box_encoder_loc, batch_size=32)

        self.trackargs = trackargs
        metric = nn_matching.NearestNeighborDistanceMetric("cosine",self.trackargs["max_cos_dist"], self.trackargs["nn_budget"])
        self.tracker = Tracker(metric)
        self.seq_info = None
        self.vis = None
        self.output_seg = None
        self.boxes = None
        self.depth = None
        print("\t + Starting on #%d sequence " % (self.reset_num))



    def predict_tlwh(self):

        """Get prediction for next frame in bounding box format `(top left x, top left y,
        width, height)`.

        Returns
        -------
        List[ndarray]
            List of bounding box.

        """
        ret = []

        for mean in self.pred_means:
            box = mean[:4].copy()
            box[2] *= box[3]
            box[:2] -= box[2:] / 2
            ret.append(box)

        return ret


    def predict_tlbr(self):

        """Get prediction for next frame in bounding box format `(min x, miny, max x,
        max y)`.

        Returns
        -------
        List[ndarray]
            List of bounding box.

        """
        ret = []

        for mean in self.pred_means:
        	box = mean[:4].copy()
        	box[2] *= box[3]
        	box[:2] -= box[2:] / 2
        	box[2:] = box[:2] + box[2:]
        	ret.append(box)

        return ret


    # def predict(self, dim):

    #     pred = self.predict_tlwh()

    #     if len(pred) >= dim:
    #         pred = pred[:dim]
    #         pred = np.asarray(pred, np.float32)
    #         pred_tensor = tf.convert_to_tensor(pred, np.float32)
    #     else:
    #         new = np.zeros((dim, 4))
    #         new[:len(pred)] = pred
    #         pred_tensor = tf.convert_to_tensor(new, np.float32)

    #     # print(pred)

    #     # init_op = tf.initialize_all_variables()

    #     # with tf.Session() as sess:
    #     #     sess.run(init_op) #execute init_op
    #     #     #print the random values that we sample
    #     #     print (sess.run(pred_tensor))

    #     return pred_tensor

    # def predict(self):

    # 	image = np.array(Image.open(os.path.join(self.loc, 'segmentation', "{0:06d}.jpg".format(self.frame_id - 1))))
    # 	return tf.convert_to_tensor(image)

    def depth2rgb(self, depth):
        #print(rgb.shape)
        depth = 255*np.array(depth, dtype=np.float32)
        r,g,b = depth, depth, depth
        rgb = np.stack((r,g,b),axis=2)
        # rgb = cv2.resize(rgb,(150,120))
        # print("rgb shape: ", np.shape(rgb))

        return rgb.astype('uint8')

    def predict(self, image_input, rgb=False):

        """Input a new frame and make detection and prediction
        """
        # image = cv2.imread(image_input, cv2.IMREAD_COLOR)
        # self.depth = scipy.misc.imread(image_input, mode='F')/255

        if rgb:
            image = image_input
            self.depth = rgb2gray(image_input)
            self.depth = np.expand_dims(image_input, 2)
        else:
            image = self.depth2rgb(image_input)
            self.depth = np.expand_dims(image_input, 2)

        start_time = time.time()

        # print "image size: ", np.array(image).shape

        shape = np.array(image).shape[:2]
        # shape = image.shape[:2]

        img = transforms.ToTensor()(image)
        img, _ = pad_to_square(img, 0)
        img = resize(img, self.img_size)

        img = Variable(img.type(self.Tensor))
        img = img.unsqueeze(0)


        with torch.no_grad():
            detections = self.detector(img)
            detections = non_max_suppression(detections, self.detargs['conf'], self.detargs['nms'])

        if detections[0] is not None:
            detections = detections[0].tolist()
            detections = rescale_boxes(detections, self.img_size, shape)
        else:
            detections = []


        self.raw_det.append(detections)
        det, detections_out = generate_detections(self.encoder, self.frame_id, detections, image.copy())
        self.feature_det.extend(detections_out)

        if self.seq_info:
            self.update_sequence_info(img)
            self.vis.update_seq_info(self.seq_info)
        else:
            if len(self.feature_det) == 0:
                dim = 129
            else:
                dim = len(self.feature_det[0]) - 10
            self.seq_info = {
                "sequence_name": "realtime",
                "image_filenames": self.img_paths,
                "detections": self.feature_det,
                "image_size": shape,
                "min_frame_idx": 0,
                "max_frame_idx": 0,
                "feature_dim": dim,
                "update_ms": self.trackargs["update_ms"]
            }
            self.vis = visualization.Visualization(self.seq_info, os.path.join(self.loc, 'trackingFrames'))

        self.vis.set_image(image.copy())

        if not os.path.exists(os.path.join(self.loc, 'trackingFrames')):
            os.makedirs(os.path.join(self.loc, 'trackingFrames'))
        if not os.path.exists(os.path.join(self.loc, 'predictionFrames')):
            os.makedirs(os.path.join(self.loc, 'predictionFrames'))
        if not os.path.exists(os.path.join(self.loc, 'segmentation')):
            os.makedirs(os.path.join(self.loc, 'segmentation'))

        self.vis.run(self.frame_callback)


        end_time = time.time()
        inference_time = datetime.timedelta(seconds=end_time - start_time)
        print("\t + Update finished on #%d frame, inference time: %s" % (self.frame_id, inference_time))

        self.frame_id += 1

        # for i in range(len(self.boxes)):
        # 	box = self.boxes[i][0]
        # 	self.boxes[i] = np.append(box, [self.depth[int(box[1]) + int(box[3]/2)][int(box[0]) + int(box[2]/2)]])
        return self.output_seg


    def reset(self):
        self.reset_num += 1
        self.frame_id = 0
        self.raw_det = []
        self.feature_det = []
        self.tracking = []
        self.track_dic = {}
        self.pred_means = None
        self.pred_covariances = None

        self.img_paths = []

        metric = nn_matching.NearestNeighborDistanceMetric("cosine",self.trackargs["max_cos_dist"], self.trackargs["nn_budget"])
        self.tracker = Tracker(metric)
        self.seq_info = None
        self.vis = None
        self.output_seg = None
        print("\t + Starting on #%d sequence " % (self.reset_num))


    # private method
    def update_sequence_info(self, img):

        """Update sequence information store in the data structure
        """

        # self.seq_info["image_filenames"] = self.img_paths
        self.seq_info["detections"] = self.feature_det
        self.seq_info["max_frame_idx"] = self.frame_id

    # private method
    def frame_callback(self, vis, frame_idx):

        """Call back function similar to DensePeds
        """

        detections = create_detections(self.seq_info["detections"], frame_idx, self.trackargs['min_det_ht'])
        detections = [d for d in detections if d.confidence >= self.trackargs['min_conf']]

        boxes = np.array([d.tlwh for d in detections])
        scores = np.array([d.confidence for d in detections])
        indices = preprocessing.non_max_suppression(boxes, self.trackargs['nms'], scores)
        detections = [detections[i] for i in indices]

        # self.pred_means, self.pred_covariances = self.tracker.predict()

        image = vis.viewer.image
        # self.save_vis(image.copy(), vis, self.track_dic, frame_idx, os.path.join(self.loc, 'predictionFrames'))

        # self.save_vis(image.copy(), vis, self.track_dic, frame_idx, os.path.join(self.loc, 'segmentation'), seg=True)
        self.tracker.update(detections)

        if self.display:
            self.save_vis(image.copy(), vis, self.track_dic, frame_idx, os.path.join(self.loc, 'trackingFrames'))

        for track in self.tracker.tracks:
            if not track.is_confirmed():
                continue
            bbox = track.to_tlwh()
            self.tracking.append([frame_idx, track.track_id, bbox[0], bbox[1], bbox[2], bbox[3]])


    def save_vis(self, image, vis, track_dic, frame_idx, loc, seg=False):

        # image = cv2.imread(self.seq_info["image_filenames"][frame_idx], cv2.IMREAD_COLOR)
        r = image.shape[0]
        c = image.shape[1]
        shp = (r, c, 1)
        c_red, c_green, c_blue = cv2.split(image)
        alpha = 0.5
        alphachn = np.ones(shp, dtype=np.uint8) * int(alpha * 255)
        image = cv2.merge((c_red, c_green, c_blue, alphachn))
        if seg:
            img = np.zeros(np.shape(image)[:-1], dtype=np.float64)
            img.fill(1)
            for track in self.tracker.tracks:
                box = track.to_tlwh().astype(np.int)
                lec = max(0,int(box[0]))
                ric = min(int(box[0] + box[2]),c-1)
                dor = max(0,int(box[1]))
                upr = min(int(box[1] + box[3]),r-1)
                cer = max(0,min(int(box[1]) + int(box[3]/2),r))
                cec = max(0,min(int(box[0]) + int(box[2]/2),c))
                # img[dor:upr,lec:ric] = self.depth[cer,cec]
            # vis.set_image(img)
            # boxes = vis.draw_trackers(track_dic, frame_idx, self.tracker.tracks, -1, self.depth)
        else:
            vis.set_image(image.copy())
            boxes = vis.draw_trackers(track_dic, frame_idx, self.tracker.tracks)
        if self.display:
            cv2.imwrite(os.path.join(loc, str(self.reset_num) + "-{0:06d}.jpg".format(frame_idx)), vis.viewer.image)
        if seg:
            # self.output_seg = rgb2gray(np.asarray(vis.viewer.image))
            self.output_seg = img #* 5


class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("/my_processed_image",ImageMsg)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/jackal/camera/image_raw",ImageMsg,self.callback)

    detargs = {"conf" : 0.8, 'nms' : 0.4}
    trackargs = {
        "update_ms" : 5,
        "min_conf" : 0.2,
        "nms" : 1.0,
        "min_det_ht" : 0,
        "max_cos_dist" : 0.2,
        "nn_budget" : 100
    }

    self.model = Memory(detargs, trackargs) # Initialize memory object



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Use bgr8 if color image is used, 32FC1 for depth image
    except CvBridgeError as e:
      print(e)

    # ret = self.model.predict(cv_image, True)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1")) # Use bgr8 if color image is used
    # except CvBridgeError as e:
    #   print(e)


def main(args):
  ic = image_converter() # image_converter object initialized
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
