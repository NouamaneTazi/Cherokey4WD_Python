#########################################################################
#
# Enseignement d'intégration ST5 CENTRALESUPELEC
# auteurs : Koen de Turck & Philippe Benabes
# Programme de serveur internet
#
# ce programme reçoit les messages depuis le robot et le controleur
# il permet de forwarder les messages depuis le controleur vers les robots
# les messages sont envoyés à tous les robots. Les robots filtrent les messages
# qui leur sont destinés
#
#########################################################################

# import des bibliothèques nécessaires au projet
import zmq
import sys

from calcul_iti import liste_commandes

# liste des noeuds connectés au serveur
nodes={}
# adresse IP du serveur
ip = "192.168.137.1"
if len(sys.argv) > 1:
	ip = sys.argv[1]

# connection pour recevoir les messages depuis robot et controleur
ctx = zmq.Context()
repaddr="tcp://{}:5005".format(ip)
repsock = ctx.socket(zmq.REP)
repsock.bind(repaddr)

# connection pour publier les messages à tous les robots
pubaddr="tcp://{}:6006".format(ip)
pubsock = ctx.socket(zmq.PUB)
pubsock.bind(pubaddr)

# def obstacle_detected(pos_obstacle,pos_bot):
#     msg1 = msg["msg"]
#     msg1["to"] = msg["forward_to"]
#     pubsock.send_pyobj(msg1)

#############################################
# procedure pour répondre aux messages reçus
#############################################
def process_msg(msg):
    reply={"all":"is fine"}
    ##################################################
    # réponse à la première connection de chaque robot
    ##################################################
    if "from" in msg and msg["from"] not in nodes:
        print("new node signing in, adding {} to nodes".format(msg["from"]))
        nodes[msg["from"]] = ""
        
    ###################################################################################
    # réponse à la demande nodelist -> renvoie la liste des objets connectés au serveur
    ###################################################################################
    if "nodelist" in msg:
       reply["nodelist"]=list(nodes.keys())+["bot001"]

    ##################################################################################
    # réponse à une demande de forward -> le message est publié à tous les robots
    ##################################################################################
    if "from" in msg and msg["from"]=="controller" and "forward_to" in msg and "coordinates" in msg:
        print("receiving coordinates")
        coor = msg["coordinates"]
        liste_com , path = liste_commandes( coor ) #TODO DEMANDE POS DU ROBOT
        print(liste_com)
        msg1={"liste_commandes":liste_com ,"liste_noeuds":path , "to": msg["forward_to"], "command": "goto"}
        pubsock.send_pyobj(msg1)

    elif "from" in msg and msg["from"]=="controller" and "forward_to" in msg and "commands" in msg:
        print("receiving commands")
        liste_com = msg["commands"]
        print(liste_com)
        msg1={"liste_commandes":liste_com, "to": msg["forward_to"], "command": "takecommands"}
        pubsock.send_pyobj(msg1)


    elif "from" in msg and msg["from"]=="controller" and "forward_to" in msg and "always_forward" in msg:
        print("receiving always forward")
        msg1={"to": msg["forward_to"], "command": "always_forward"}
        pubsock.send_pyobj(msg1)

    elif "from" in msg and msg["from"]=="controller" and "msg" in msg and "forward_to" in msg:
        print("forwarding a controller msg to ",msg["forward_to"])
        msg1 = msg["msg"]
        msg1["to"] = msg["forward_to"]
        pubsock.send_pyobj(msg1)




    ###################################################################
    #  réponse a une requete
    ###################################################################
    if "request" in msg:
        print("the bot has a request")
        reply={"all":"is fine"}

    #####################################################################
    # réponse à un message d'information du robot
    #####################################################################
    if "info" in msg:
        if  msg["info"]=="obstacle" :   
                numbot=msg["from"]
                print("the bot" ,numbot, "has an obstacle")

                # pos_obstacle , pos_bot = msg["pos_obstacle"] , msg["pos_bot"]
                # obstacle_detected(pos_obstacle,pos_bot)

        if  msg["info"]=="clr_obstacle" :
                numbot=msg["from"]
                print("the bot" ,numbot, "no longer has an obstacle")

    # renvoie le message de réponse 
    return reply

###############################################################
# boucle de réception des messages et transmission des réponses
###############################################################

while True:
    # reception du message
    msg = repsock.recv_pyobj()
    #print("received: {}".format(msg))

    # calcul de la réponse
    reply = process_msg(msg)
    #print("reply: {}".format(reply))

    # transmission de la réponse aux robots
    repsock.send_pyobj(reply)

