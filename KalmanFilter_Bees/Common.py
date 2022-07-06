# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 11:31:55 2021

@author: Adrien
"""

import cv2
import numpy as np

def mediane_image(video, nbr):
    cap=cv2.VideoCapture(video)
    tab_image=[]
    for f in range(nbr):
        ret, frame=cap.read()
        if ret is False:
            break
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)      #Affiche l'image en niveau de gris
        tab_image.append(image)
    tab_image=np.array(tab_image)
    cap.release()
    return np.percentile(tab_image,50, axis=0)                  #mediane de l'array tab_image

def calcul_mask(image, fond, seuil):                            #Retourne une image binaire qui affiche les abeilles en blanc sur fond noir
    image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width=image.shape
    mask=np.zeros([height, width], np.uint8)
    image=image.astype(np.int32)
    mask[:,-200:-1]=0
    for y in range(height-200):
        for x in range(width):
            if abs(fond[y][x]-image[y][x])>seuil:
                mask[y][x]=255
    mask[:,-200:-1]=0
    kernel=np.ones((5, 5), np.uint8)
    mask=cv2.erode(mask, kernel, iterations=1)
    mask=cv2.dilate(mask, kernel, iterations=4)
    return mask