#########################################################################
#
# Enseignement d'intégration ST5 CENTRALESUPELEC
# auteurs : Koen de Turck & Philippe Benabes
# Programme de client de controle et d'interface homme machine
#
# ce programme sert d'interface homme machine et permet d'envoyer des
# messages au robot via le serveur
#
#########################################################################

# import des bibliothèques nécessaires au projet
import zmq
import msvcrt
import time


ctx, addr, sock = 3*[None]
# n° d'IP du serveur
ip = "192.168.137.1"

# procedure de connexion au serveur
def start(ip=ip):
    global ctx, addr, sock
    ctx = zmq.Context()
    addr="tcp://{}:5005".format(ip)
    sock = ctx.socket(zmq.REQ)
    sock.connect(addr)

# procedure de récupération des noms des clients connectés au serveur
def get_nodes():
	print('sending botlist request to server @ {} ...'.format(addr))
	sock.send_pyobj({"from":"controller", "nodelist":""})
	msg = sock.recv_pyobj()
	return msg["nodelist"]

# procedure pour forwarder un message à un robot via le serveur
def forward_msg(bot, msg):
	print('relaying msg to bot {} via server @ {} ...'.format(bot, addr))
	sock.send_pyobj({"from":"controller", "forward_to":bot, "msg": msg})
	msg = sock.recv_pyobj()
	return msg
def send_coordinates(bot,coor):
    print('sending coordinates to server')
    sock.send_pyobj({"from":"controller", "forward_to":bot, "coordinates": coor})
    msg = sock.recv_pyobj()
    return msg
def send_commands(bot,liste_commandes):
    print('sending commands to server')
    sock.send_pyobj({"from":"controller", "forward_to":bot, "commands": liste_commandes})
    msg = sock.recv_pyobj()
    return msg

def always_forward(bot):
    print('sending always forward to server')
    sock.send_pyobj({"from":"controller", "forward_to":bot, "always_forward": True})
    msg = sock.recv_pyobj()
    return msg
######################################################
#
# programme principal
#
######################################################

# connexion au serveur
start(ip)

# variable indiquant quand il faut afficher le message d'accueil
endcmd=1
while True :
    if (endcmd):
        #
        # affichage du message d'accueil de l'IHM
        # après chaque appui de touche
        #
        print("Programme principal");
        print("Robots disponibles",get_nodes());
        print("Entrez votre commande")
        endcmd=0

    #
    # detection de l'appui d'une touche sur le clavier
    #
    if (msvcrt.kbhit()>0):
        command=msvcrt.getch()
        #
        # traitement des commandes clavier
        #
        if (command==b'z' or command==b'Z'):
            # envoi d'une commande start au robot bot1
            msg={"command":"start"}
            forward_msg("bot001", msg)
        if (command==b's' or command==b'S'):
            msg={"command":"backwards"}
            forward_msg("bot001", msg)
        if (command==b'd' or command==b'D'):
            msg={"command":"right"}
            forward_msg("bot001", msg)
        if (command==b'q' or command==b'Q'):
            msg={"command":"left"}
            forward_msg("bot001", msg)
        if (command==b'j' or command==b'J'):
            msg={"command":"servo"}
            forward_msg("bot001", msg)
        if (command==b'n' or command==b'N'):
            coor = eval(input("Enter destination coordinates:"))
            send_coordinates("bot001", coor)
        if (command==b'c' or command==b'C'):
            liste_commandes = list(eval(input("Enter commands:")))
            send_commands("bot001", liste_commandes)

        if (command==b'o' or command==b'O'):
            msg={"command":"always_forward"}
            always_forward("bot001")
        if (command==b'p' or command==b'P'):
            msg={"command":"always_forward"}
            always_forward("bot002")



        if (command==b'm' or command==b'M'):
            liste_commandes = list(eval(input("Enter commands:")))
            send_commands("bot002", liste_commandes)
        if (command==b'b' or command==b'B'):
            # envoi d'une commande start au robot bot2
            msg={"command":"start"}
            forward_msg("bot002", msg)
        if (command==b'e' or command==b'E'):
            # envoi d'une commande stop au robot bot1
            msg={"command":"stop"}
            forward_msg("bot001", msg)
        if (command==b'f' or command==b'F'):
            # envoi d'une commande stop au robot bot2
            msg={"command":"stop"}
            forward_msg("bot002", msg)
        endcmd=1

    # attente de 1/10 de seconde dans la boucle
    time.sleep(0.1)
        

