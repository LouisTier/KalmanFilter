# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 11:31:29 2021

@author: Adrien
"""

#faire variance sur les couleurs

import cv2
import numpy as np
from Common import mediane_image 
from Common import calcul_mask 
from PIL import Image as img
from KalmanFilterAbeille import *


video='abeilles.avi'
image_fond="img-0.png"
couleur_positions=(0, 0, 255)
couleur_prediction=(0, 255, 0)

distance_mini = 350
nbr_old=0
abeille=0
seuil=100

fond=mediane_image(video, seuil)

cv2.imshow('fond', fond)

cap=cv2.VideoCapture(video)

objets_KF=[]
objets_points=[]

def distance(point, liste_points):
    distances=[]
    for p in liste_points:
        distances.append(np.sum(np.power(p-np.expand_dims(point, axis=-1), 2)))
    return distances

while True:
    ret, frame1=cap.read()  
    frame = frame1
                
    rgb_planes = cv2.split(frame)

    result_planes = []
    result_norm_planes = []
    
    ##Attenuation des ombres pour qu'elles ne soient plus reconnues
    el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))
    for plane in rgb_planes:
        dilated_img = cv2.dilate(plane, el)                 #np.ones((8,8), np.uint8)
        bg_img = cv2.medianBlur(dilated_img, 21)
        diff_img = 255 - cv2.absdiff(plane, bg_img)
        norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        result_norm_planes.append(norm_img)

    frame = cv2.merge(result_norm_planes)
     

       
    tickmark=cv2.getTickCount()
    mask=calcul_mask(frame, fond, seuil)
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    nbr=0
    
  
    points = []
    for e in elements:
        ((x, y), rayon)=cv2.minEnclosingCircle(e)
        objets_KF.append(KalmanFilter(cv2.getTickFrequency()/(cv2.getTickCount()-tickmark), [x,y], [rayon//2,rayon//2]))
        predictions = objets_KF[-1].predict().tolist()
        
        if rayon<100 and rayon>20:
            points.append([int(x),int(y),rayon//2,rayon//2])
            objets_KF.append(KalmanFilter(cv2.getTickFrequency()/(cv2.getTickCount()-tickmark), [x,y], [rayon//2,rayon//2]))
            predictions = objets_KF[-1].predict().tolist()
            
            
            cv2.rectangle(frame1, (int(x)-int(rayon//2),int(y)-int(rayon//2)), (int(x)+int(rayon//2),int(y)+int(rayon//2)), couleur_positions, 2)
            cv2.circle(frame1, (int(x),int(y)), 1, (0,255,0),3)
            
            cv2.rectangle(frame1, (int(predictions[0][0])-int(predictions[4][0]//2),int(predictions[1][0])-int(predictions[4][0]//2)), (int(predictions[0][0])+int(predictions[4][0]//2),int(predictions[1][0])+int(predictions[4][0]//2)), couleur_prediction, 2)
            cv2.arrowedLine(frame1,
                        (int(predictions[0][0]), int(predictions[1][0])),
                        (int(predictions[0][0]+3*predictions[2][0]), int(predictions[1][0]+3*predictions[3][0])),
                        color=(0, 0, 255),
                        thickness=2,
                        tipLength=0.2)
            
    
            
         # calcul des distances
    nouveaux_objets=np.ones(len(np.expand_dims(points, axis=-1)))
    tab_distances=[]
    if len(objets_points):
        for point_id in range(len(points)):
            distances=distance(points[point_id], objets_points)
            tab_distances.append(distances)

        tab_distances=np.array(tab_distances)
        sorted_distances=np.sort(tab_distances, axis=None)

        for d in sorted_distances:
            if d>distance_mini:
                break
            id1, id2=np.where(tab_distances==d)
            if not len(id1) or not len(id2):                                    #si tout les deux vides retourne au début du prochain tour de boucle
                continue
            tab_distances[id1, :]=distance_mini+1
            tab_distances[:, id2]=distance_mini+1
            objets_KF[id2[0]].update(np.expand_dims(points[id1[0]], axis=-1))
            nouveaux_objets[id1]=0   
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            nbr+=1
    if nbr>nbr_old:
        abeille+=1
    nbr_old=nbr
    fps=cv2.getTickFrequency()/(cv2.getTickCount()-tickmark)
    cv2.putText(frame, "FPS: {:05.2f}  Seuil: {:d}".format(fps, seuil), (10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, couleur_positions, 1)
    cv2.imshow('video', frame1)   #changer par frame1 pour voir le suivi
    cv2.imshow('mask', mask)
    key=cv2.waitKey(1)&0xFF
    if key==ord('q'):
        break
    if key==ord('p'):
        seuil+=1
    if key==ord('m'):
        seuil-=1
    if key==ord('a'):
        for cpt in range(20):
            ret, frame1=cap.read()   

cap.release()
cv2.destroyAllWindows()