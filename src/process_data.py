import sys
import rospkg
import os
import cPickle
import cv2
from cv_bridge import CvBridge

if __name__ == '__main__':
    bridge = CvBridge()
    rospack = rospkg.RosPack()
    root = os.path.join(rospack.get_path('apple_grasper'), 'data')
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    else:
        file_name = 'grasper_20191009_160416.pickle'

    with open(os.path.join(root, file_name), 'rb') as fh:
        data = cPickle.load(fh)

    # Hard-coded stuff here for testing
    import pdb
    pdb.set_trace()
    img_topic = '/camera/color/image_raw/compressed'
    img = bridge.compressed_imgmsg_to_cv2(data[img_topic][0])
