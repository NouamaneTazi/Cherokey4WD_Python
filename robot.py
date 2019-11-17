#########################################################################
#
# Enseignement d'intégration ST5 CENTRALESUPELEC
# auteurs : Koen de Turck & Philippe Benabes
# Programme de client tournant sur les robots
#
# ce programme sert à controler les robots : réception des messages
# et transmission des messages au serveur
#
#########################################################################

# import des bibliothèques nécessaires au projet
from __future__ import division, print_function
import zmq
import sys
import random

import logging
import signal
import threading
import time
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# import de la bibliothèque liaison série   
from robust_serial import write_order, Order, write_i8, read_i16, read_i8
from robust_serial.threads import CommandThread, ListenerThread
from robust_serial.utils import open_serial_port
from constants import BAUDRATE

from image_processing import processimage
manual = 0
use_ultrasonic = 1
use_camera = 1 
record_camera_time = 0 #0.49 qd tout travaille
path=0
#######################################################
# initialisation de la communication avec la raspberry
#######################################################
def init_raspberry():
    serial_file = None
    try:
        # ouvre la liaison série avec l'arduino
        serial_file = open_serial_port(baudrate=BAUDRATE)
        print(serial_file)
    except Exception as e:
        print('exception')
        raise e

    is_connected = False
    # Initialize communication with Arduino
    while not is_connected:
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        #print(bytes_array)
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    return serial_file

# recupere l'identifiant pour la liaison série avec la raspberry
serial_file=init_raspberry()

# N° d'IP du serveur
ip = "192.168.137.1"
if len(sys.argv) > 1:
	ip = sys.argv[1]
	
# nom du robot
name='bot001'
if len(sys.argv) > 2:
	name = sys.argv[2]

#############################################
# connection initisale du robot sur le serveur
#############################################
ctx = zmq.Context()
reqaddr="tcp://{}:5005".format(ip)
reqsock = ctx.socket(zmq.REQ)
reqsock.connect(reqaddr)

####################################################################
# envoie le message initisal pour enregistrer le robot sur le serveur
####################################################################
print("initisal hello msg ...")
# envoi d'un message de connection (avec uniquement un 'from')
msg={"from":name}
reqsock.send_pyobj(msg)
# recupération de la réponse
rep=reqsock.recv_pyobj()
print("received:{}".format(rep))

##############################################
# création du serveur en mode écoute pour
# répondre aux sollicitations du serveur
##############################################
subaddr="tcp://{}:6006".format(ip)
subsock = ctx.socket(zmq.SUB)
subsock.connect(subaddr)
subsock.setsockopt(zmq.SUBSCRIBE, b"")



liste_commandes=[]
liste_noeuds=[]

#######################################################
# procedure pour répondre aux sollicitations du serveur
#######################################################
def process_msg(msg):
    global cur_pos
    global liste_commandes , path
###############################################################
    # verifie si le message est destiné au robot
    # s'il y a un champ 'to' et que le nom n'est pas celui du robot
    # on oublie le message
    ###############################################################
    if "to" in msg and msg["to"] !=name:
        print("this msg wasn't for me...")
        
    ##############################################################
    # sinon traitement des différents messages reçus
    ###############################################################
    else:
        if "command" in msg :
                #########################################
                # quand on reçoit un start
                #########################################
                vmax = 60
                if msg["command"]=="start":
                        print("the server orders me to start")
                        write_order(serial_file, Order.MOTOR)
                        write_i8(serial_file, vmax)  # vitesse du moteur de droite
                        write_i8(serial_file, vmax)  # vitesse du moteur de gauche
                        
                if msg["command"]=="backwards":
                        print("the server orders me to go backwards")
                        write_order(serial_file, Order.MOTOR)
                        write_i8(serial_file, -100)  
                        write_i8(serial_file, -100)  
                                
                if msg["command"]=="right":
                        print("the server orders me to go right")
                        write_order(serial_file, Order.MOTOR)
                        write_i8(serial_file, -100)  
                        write_i8(serial_file, 100)                  
                if msg["command"]=="left":
                        print("the server orders me to go right")
                        write_order(serial_file, Order.MOTOR)
                        write_i8(serial_file, 100)  
                        write_i8(serial_file, -100)
                        
                if msg["command"]=="goto" :
                        print("the server orders me to go to destination")
                        liste_commandes = msg["liste_commandes"]
                        #liste_commandes.append('s')
                        liste_noeuds = msg["liste_noeuds"]
                        print(liste_commandes)
                if msg["command"]=="takecommands" :
                        print("the server orders me to take his commands")
                        liste_commandes = list(msg["liste_commandes"])
                        print(liste_commandes)
                if msg["command"]=="always_forward" :
                        print("the server orders me to go forward always")
                        liste_commandes = ["t"]*100
                        path = 8

                if msg["command"]=="servo":
                        print("the server orders SERVO")
                        write_order(serial_file, Order.SERVO)
                        write_i8(serial_file, 60)  
                        
                #########################################
                # quand on reçoit un stop
                #########################################
                if msg["command"]=="stop" :
                        print("the server orders me to stop")
                        liste_commandes = []
                        write_order(serial_file, Order.STOP)
                        cur_pos = [0,-1]


############################################################
# boucle principale d'écoute et de traitement des évènements
############################################################


def move(cx):
    erreur=cx-160
    xmax=160
    vmax_droit=55 #55 marche pour les croix
    v_min_turn=90 #90
    v_max_turn=92 #95
    err_norm=erreur/xmax
    lim=0.35
    if 0<err_norm<lim:
        v_droite=int(vmax_droit*(1-err_norm))
        v_gauche=vmax_droit

    elif -lim<=err_norm<0:
        v_droite=vmax_droit
        v_gauche=int(vmax_droit*(1-err_norm))
    
    elif err_norm>lim:
        lmbda = (err_norm-lim)/(1-lim)
        v_droite=-int(v_min_turn * lmbda + v_max_turn * (1-lmbda) )
        v_gauche= int(v_min_turn * lmbda + v_max_turn * (1-lmbda) )
    
    else:
        lmbda = (abs(err_norm)-lim)/(1-lim)
        v_droite= int(v_min_turn * lmbda + v_max_turn * (1-lmbda) )
        v_gauche= -int(v_min_turn * lmbda + v_max_turn * (1-lmbda) )
    #print("erreur norm = %0.2f"%(err_norm*100),"%")
    #print('v_gauche=%d , v_droite=%d'%(v_gauche,v_droite))
    write_order(serial_file, Order.MOTOR)
    write_i8(serial_file,v_droite)  # vitesse du moteur de droite
    write_i8(serial_file, v_gauche )  # vitesse du moteur de gauche

def update_position():
    global direction , cur_pos
    if direction ==0: #Nord 
            cur_pos[1] += 1
    
    elif direction ==2: #Sud 
            cur_pos[1] -= 1
    
    elif direction ==1: #Est 
            cur_pos[0] += 1
    
    elif direction ==3: #Ouest 
            cur_pos[0] -= 1
    print("UPDATE POSITION: ",cur_pos)
    


def demi_tour():
    global direction
    direction=(direction+2)%4
    v=100
    write_order(serial_file, Order.MOTOR)
    write_i8(serial_file, v)  # vitesse du moteur de droite
    write_i8(serial_file, -v)  # vitesse du moteur de gauche
    time.sleep(3)
    write_order(serial_file, Order.STOP)
        
def traite_commande(commande):
    global direction , cur_pos 
    turn_time=2.3 
    if commande == "d":
        direction=(direction+1)%4 #sens horaire
        v=100
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -v)  # vitesse du moteur de droite
        write_i8(serial_file, v)  # vitesse du moteur de gauche
        time.sleep(turn_time)
    elif commande == "g":
        direction=(direction-1)%4
        v=100
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, v)  # vitesse du moteur de droite
        write_i8(serial_file, -v)  # vitesse du moteur de gauche
        time.sleep(turn_time)
    elif commande == "t":
        v=60
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, v)  # vitesse du moteur de droite
        write_i8(serial_file, v)  # vitesse du moteur de gauche
    elif commande == "turn":
        demi_tour()
    elif commande == 's':
        demi_tour()
        write_order(serial_file, Order.STOP)
    update_position()
flag_obstacle = 0
capteur = 0
count_obstacle = 0

def change_path():
    global liste_commandes
    if len(liste_commandes)>0 :
        premiere_commande = liste_commandes.pop(0)
        D={"t":"g", "d":"t", "g":"turn", "s":"s"}
        G={"t":"d", "d":"turn", "g":"t", "s":"s" }
        if direction ==0: #Nord
            if cur_pos[0]-1 >= 0:
                liste_commandes = ["d","d","d"] + [D[premiere_commande]] + liste_commandes

            else:
                liste_commandes = ["g","g","g"] + [G[premiere_commande]] + liste_commandes
        elif direction ==2: #Sud
            if cur_pos[0]+1 <= 4:
                liste_commandes = ["d","d","d"] + [D[premiere_commande]] + liste_commandes

            else:
                liste_commandes = ["g","g","g"] + [G[premiere_commande]] + liste_commandes
        elif direction ==1 : #Est
            if cur_pos[0]+1 <= 4:
                liste_commandes = ["d","d","d"] + [D[premiere_commande]] + liste_commandes

            else:
                liste_commandes = ["g","g","g"] + [G[premiere_commande]] + liste_commandes

        elif direction ==3: #Ouest
            if cur_pos[0]-1 >= 0:
                liste_commandes = ["d","d","d"] + [D[premiere_commande]] + liste_commandes

            else:
                liste_commandes = ["g","g","g"] + [G[premiere_commande]] + liste_commandes





def detect_ultrasonic():
    global flag_obstacle,capteur,count_obstacle
    #print("flag_obstacle ",flag_obstacle)
    capteur = read_i8(serial_file) # recupere la positison du capteur
    #print(capteur)
    if capteur==4 and flag_obstacle==0 :     # s'il y a un obstaccle
        flag_obstacle=1 # on bloque jusqu'à ce qu'il n'y ait plus d'obstacle
        print("obstacle detecte ...")
        write_order(serial_file, Order.STOP)
        time.sleep(5)
##        msg={"from":name,"info":"obstacle"}
##        reqsock.send_pyobj(msg)
##        rep=reqsock.recv_pyobj()
    if capteur==4 and flag_obstacle==1 and path!=8:
        change_path()
        demi_tour()

    if (capteur==100 and flag_obstacle==1): # si on est sur qu'il n'y a plus d'obstacle
        flag_obstacle=0                   # on debloque jusqu'au nouvel obstacle
        #print("obstacle cleared ...")
##        msg={"from":name,"info":"clr_obstacle"}
##        reqsock.send_pyobj(msg)
##        rep=reqsock.recv_pyobj()

camera = PiCamera()
camera.resolution = (320, 240) #640 : 0.53 //320: 0.48
camera.framerate = 90 # 70-90 : 0.43 // 


cur_pos = [0,-1]
ref_direction=["Nord","Est","Sud","Ouest"]
direction = 0
counter_intersection = 0
counter_pas_intersection=0
count_timer , sum_delta = 0 , 0
while True:
    # ecoute des messages avec timeout
    i = subsock.poll(timeout=0)
    # s'il y a un message reçu on le traite
    if i != 0:
         msg = subsock.recv_pyobj()
         #print("received: {}".format(msg))
         process_msg(msg)
    # s'il n'y a pas de message on traite le reste des opérations du robot
    if len(liste_commandes)==0:
        manual = 1
    else:
        manual = 0
        
        
    # on simule une info capteur par un message aléatoire
    if use_ultrasonic == 1 and serial_file.in_waiting >=1 :    # si on a un octet present sur la liaison série 
        #Detect Ultrasonic radar (URM Sensor)
        detect_ultrasonic()


    if use_camera==1:
        #Detect Centroid and intersection
        if record_camera_time : start = time.time()
        
        rawCapture = PiRGBArray(camera)
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        result = processimage(image)
        #result=[]
        
        if record_camera_time :
            finish = time.time()
            count_timer+=1
            sum_delta = (sum_delta + finish - start)
            moy = sum_delta/count_timer
            print('Captured at %.2ffps' % moy)
        
        if "centroid" in result and manual==0:
            cx,cy=result["centroid"]
            move(cx)

        if "intersection" in result and result["intersection"] and manual==0: # on se trouve sur arrete, on doit executer command
            counter_intersection += 1
            counter_pas_intersection=0
            #print("+ INTERSECTION !",counter_intersection)
        
        elif "intersection" in result and result["intersection"]==False and manual ==0:  # y a pas d'intersection, on se trouve sur intersection
            counter_pas_intersection+=1
            #print("- PAS D'INTERSECTION",counter_pas_intersection)
            if counter_intersection >=4 : # 2 pour path 1, 4 pour quadrillage
                 #Surement arrivés à une intersection
                if counter_pas_intersection>=2:
                    update_position()
                    counter_intersection = 0
                    counter_pas_intersection=0
                    print("FIN INTERSECTION REALISE COMMANDE")
                    print(liste_commandes)
                    commande = liste_commandes.pop(0)
                    traite_commande(commande)
            else:        
                counter_intersection=0
        
    
                

                

