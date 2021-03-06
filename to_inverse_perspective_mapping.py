# coding: utf-8
# Inverse Perspective Mappingを確認する
#%matplotlib inline
import cv2
from matplotlib import pyplot as plt
import numpy as np
import time
import os
import sys
import math
from cv_functions import *

def main():
    FILE_DIR = '../mycar/data/tub_1_20-08-13'
    FILENAME = "1_cam-image_array_"
    OUTPUT_DIR ='./output'
    mkdir(OUTPUT_DIR)
    print("OpenCV Version : %s " % cv2.__version__)
    try:
        IMAGE_FORMAT = 1
        cv_bgr = cv2.imread(os.path.join(FILE_DIR, FILENAME)+".jpg", IMAGE_FORMAT)

        ########################################
        # Inverse Perspective Mapping Coordinates
        ########################################
        # robocar camera demo_lane
        ipm_vertices = calc_ipm_vertices(cv_bgr,
                                         #top_width_rate=0.80,top_height_position=0.65,
                                         top_width_rate=0.28,top_height_position=0.45,
                                         bottom_width_rate=2.0,bottom_height_position=1)

        ########################################
        # IPM座標を確認する
        ########################################
        cv_bgr_ipm_before_preview = draw_vertices(cv_bgr,ipm_vertices)
        plt.title('Before IPM')
        plt.imshow(to_rgb(cv_bgr_ipm_before_preview))
        plt.show()
        cv_bgr_ipm_after_preview = to_ipm(cv_bgr_ipm_before_preview,ipm_vertices)
        plt.title('After IPM')
        plt.imshow(to_rgb(cv_bgr_ipm_after_preview))
        plt.show()
        cv2.imwrite(OUTPUT_DIR+"/result_"+FILENAME+"_before_ipm.jpg",cv_bgr_ipm_before_preview)
        cv2.imwrite(OUTPUT_DIR+"/result_"+FILENAME+"_after_ipm.jpg",cv_bgr_ipm_after_preview)
        ########################################
        # Inverse Perspective Mapping
        ########################################
        cv_bgr = to_ipm(cv_bgr,ipm_vertices)
        plt.title('IPM')
        plt.imshow(to_rgb(cv_bgr))
        plt.show()
        cv2.imwrite(OUTPUT_DIR+"/result_"+FILENAME+"_ipm.jpg",cv_bgr)
    except:
        import traceback
        traceback.print_exc()
    finally:
        pass
    return

if __name__ == '__main__':
    main()

