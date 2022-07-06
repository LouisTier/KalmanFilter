# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 11:26:35 2021

@author: Adrien

#On affiche le fond avec le filtre médian 
"""
import cv2
import numpy as np
from Common import mediane_image




image=mediane_image('abeilles.avi', 100)
cv2.imshow('fond', image.astype(np.uint8))

cv2.waitKey()
#cap.release()
cv2.destroyAllWindows()
