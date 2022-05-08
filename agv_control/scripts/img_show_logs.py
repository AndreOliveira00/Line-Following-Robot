#!/usr/bin/env python

# Authors: André Oliveira nº1181045 #
# Authors: Tiago Cunha nº1180922 #

import pickle
import numpy as np
import cv2 as cv
import os
import numpy as np
from cv_bridge import CvBridge,CvBridgeError

NUMBER_LOGS=300           

bridge= CvBridge()

for num in range(1,NUMBER_LOGS):
    with open(os.path.expanduser('~')+"/logs/IMG/img_%d"%num, "rb") as fp:   # Unpickling
        b = pickle.load(fp)
        np_arr=np.frombuffer(b,np.uint8)
        img=cv.imdecode(b,cv.IMREAD_COLOR)
        #print(img)
        #   print(len(b))
        #cv_image = bridge.imgmsg_to_cv(b,'bgr8')
        cv.imshow("ok",img)
        #cv.ShowImage("Image view",cv_image)
        cv.waitKey(0)
