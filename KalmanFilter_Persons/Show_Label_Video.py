# -*- coding: utf-8 -*-
"""
Created on Tue Oct 12 19:03:03 2021

@author: Louis

CE CODE AFFICHE LA VIDEO AVEC LE DETECTEUR DU FICHIER TEXTE
"""

import cv2
import numpy as np
from numpy import genfromtxt
import os
import glob

datasets=r"C:\Users\Adrien\Documents\KALMAN"
dataset="ETH-BAHNHOF"

dir_images=datasets+"/"+dataset+"/img1/"
fichier_label=datasets+"/"+dataset+"/gt/gt.txt"

if not os.path.exists(fichier_label):
    print("Le fichier de label n'existe pas ...", fichier_label)
    

data=genfromtxt(fichier_label, delimiter=',')
id_frame=0
id_objet=0

for image in glob.glob(dir_images+"*.jpg"):
    frame=cv2.imread(image)

    mask=data[:, 0]==id_frame
    for d in data[mask, :]:             #On affiche les rectangles du detecteur
        cv2.rectangle(frame, (int(d[2]), int(d[3])), (int(d[2]+d[4]), int(d[3]+d[5])), (0, 255, 0), 2)

    cv2.imshow("frame", frame)

    key=cv2.waitKey(70)&0xFF
    if key==ord('q'):
        quit()
    id_frame+=1