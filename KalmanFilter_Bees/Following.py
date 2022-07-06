# -*- coding: utf-8 -*-
"""
Created on Tue Oct 12 19:03:36 2021

@author: Adrien
"""

import cv2
import numpy as np

import math
import os
import glob

from Common import mediane_image 
from Common import calcul_mask 
from PIL import Image as img
from KalmanFilterAbeille import *

video='abeilles.avi'          #Import de la vidéo



image_fond="img-0.png"



couleur_positions=(0, 0, 255)
couleur_prediction=(0, 255, 0)



seuil=100
distance_mini=1e9

fond=mediane_image(video, seuil)      #création du fond median 

cv2.imshow('fond', fond.astype(np.uint8))

cap=cv2.VideoCapture(video)    #Implémentation de la vidéo


#Initialisation des listes qui nous serviront à stocker les objets et les localiser
objets_points=[]
objets_id=[]
objets_KF=[]



def distance(point, liste_points):
    distances=[]
    for p in liste_points:
        distances.append(np.sum(np.power(p-np.expand_dims(point, axis=-1), 2)))
    return distances


id_objet=0          #Initialisation d'un compteur qui donnera un ID aux objets détectés


while True:
    ret, frame1=cap.read()     #frame 1 pour différencier la vrai de celle où on va faire les opérations (frame)
    frame = frame1
                
    rgb_planes = cv2.split(frame)   #divise l'image en trois plans R, G et B

    result_planes = []
    result_norm_planes = []
    
    ##Attenuation des ombres pour qu'elles ne soient plus reconnues
    el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))   #masque pour enlever les ombres (peut remplacer M)
    for plane in rgb_planes:
        M =  np.ones((7,7), np.uint8)
        dilated_img = cv2.dilate(plane, M, iterations=1)                 
        bg_img = cv2.medianBlur(dilated_img, 21)
        diff_img = 255 - cv2.absdiff(plane, bg_img)
        norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        result_norm_planes.append(norm_img)

    frame = cv2.merge(result_norm_planes)
    ##
    
    
    # Prediction de l'ensemble des objets + affichage
    
    for id_obj in range(len(objets_points)):
            predict_etat=objets_KF[id_obj].predict()               #on fait les prédictions sur tout nos objets
            predict_etat=np.array(predict_etat, dtype=np.int32)
            etat = []
            
            for i in range(len(predict_etat)):                     #On transforme le resultat en liste pour faciliter les calculs
                etat.append(predict_etat[i][0])
                
            objets_points[id_obj]=[predict_etat[0], predict_etat[1], predict_etat[4], predict_etat[5]]
            
            
            cv2.circle(frame1, (etat[0], etat[1]), 5, (0, 0, 255), 2)   #On trace le point au centre de l'objet détecté
           
            cv2.arrowedLine(frame1,                                     #On trace le vecteur vitesse prédit (on ajoute des coefficients pour bien voir les flèches à l'écran)
                            (etat[0], etat[1]),
                            (etat[0]+3*etat[2], etat[1]+3*etat[3]),
                            color=(0, 0, 255),
                            thickness=2,
                            tipLength=0.2)
            cv2.putText(frame1,                                                  #on affiche l'id
                        "ID{:d}".format(objets_id[id_obj]),
                        (int(etat[0]-etat[4]/2), int(etat[1]-etat[5]/2)),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (255, 0, 0),
                        2)

    

    # Affichage des données (rectangle) du detecteur
    points=[]                                                   
    
    tickmark=cv2.getTickCount()
    
    mask=calcul_mask(frame, fond, seuil)
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]      #C'est notre détecteur d'objet. Methode provenant d'OpenCv
    nbr=0
    for e in elements:
        ((x, y), rayon)=cv2.minEnclosingCircle(e)
        if rayon>20 and rayon<100:                                                      #On filtre "à la main" les objets détectés trop petit ou trop grand qui seraient absurde par rapport à la taille des abeilles
            cv2.rectangle(frame1, (int(x)-int(rayon),
                                   int(y)-int(rayon)),
                                  (int(x)+int(rayon),int(y)+int(rayon)),
                                  couleur_positions, 2)
            cv2.circle(frame1, (int(x),int(y)), 2, (0, 255, 0), 2)
            points.append([int(x), int(y), rayon, rayon])                               #On ajoute le point à la liste

    

    # calcul des distances
    nouveaux_objets=np.ones((len(points)))                                    #On crée une liste qui contiendra l'information si oui ou non les objets à l'écran viennent d'apparaître à cette frame
    tab_distances=[]                                                            #Tableau qui contient les distances pour chaque filtre de Kalman déjà crée (lien avec les tableaux affichés lors de la présentation)
    if len(objets_points):
        for point_id in range(len(points)):
            distances=distance(points[point_id], objets_points)               #Calcul des distances
            tab_distances.append(distances)

        tab_distances=np.array(tab_distances)
        sorted_distances=np.sort(tab_distances, axis=None)

        for d in sorted_distances:
            if d>distance_mini:
                break
            id1, id2=np.where(tab_distances==d)                                 #id1 = id du rectangle et id2 = id du filtre
            if not len(id1) or not len(id2):                                    #si tout les deux vides retourne au début du prochain tour de boucle
                continue
            tab_distances[id1, :]=distance_mini+1                               
            tab_distances[:, id2]=distance_mini+1
            objets_KF[id2[0]].update(np.expand_dims(points[id1[0]], axis=-1))
            nouveaux_objets[id1]=0              #Ce n'est pas un nouvel objet : on met l'indice de l'id1 à 0 pour montrer que l'objet était là avant

    # Création du filtre de Kalman pour les nouveaux objets
    for point_id in range(len(points)):
        if nouveaux_objets[point_id]:                   
            print("NOUVEAU", points[point_id])
            objets_points.append(points[point_id])
            objets_KF.append(KalmanFilter(cv2.getTickFrequency()/(cv2.getTickCount()-tickmark),
                                          [points[point_id][0],points[point_id][1]],
                                          [points[point_id][2],points[point_id][3]]))
            objets_id.append(id_objet)
            
            id_objet+=1

    # Nettoyage ...
    
    tab_id=[]
    for id_point in range(len(objets_points)):
        if int(objets_points[id_point][0])<-100 or \
        int(objets_points[id_point][1])<-100 or \
            objets_points[id_point][0]>frame.shape[1]+100 or \
            objets_points[id_point][1]>frame.shape[0]+100:
            print("SUPPRESSION", objets_points[id_point])
            tab_id.append(id_point)

    
    for index in sorted(tab_id, reverse=True):
        del objets_points[index]
        del objets_KF[index]
        del objets_id[index]
        

            
    #Affichage sur la video
    fps=cv2.getTickFrequency()/(cv2.getTickCount()-tickmark)
    cv2.putText(frame, "FPS: {:05.2f}  Seuil: {:d}".format(fps, seuil), (10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, couleur_positions, 1)
    cv2.imshow('video', frame1)   #changer par frame1 pour voir le suivi
    cv2.imshow('mask', mask)
    key=cv2.waitKey(1)&0xFF
    if key==ord('q'):               #Quitter la video
        break
    if key==ord('p'):               #Augmenter le seuil
        seuil+=1
    if key==ord('m'):               #Diminuer le seuil
        seuil-=1
    if key==ord('a'):               #Commande pour passer des frames
        for cpt in range(20):
            ret, frame1=cap.read()   

cap.release()
cv2.destroyAllWindows()



   