############### LIBRERIAS ###############

#Procesado de imagenes
import imutils
import cv2

#Sockets
import socket

#Servos
import Adafruit_PCA9685

#Puerto serie
import serial

#Otras librerias
import math
import time
import random
import io
import os
import sys
import subprocess

###### INICIALIZACION DE VARIABLES ######

#Socket

#Servidor
hostname = '0.0.0.0'
hostport = 10000
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((hostname, hostport))

#Cliente
server.listen(1)
client, addr = server.accept()

#Se espera hasta obtener el primer caracter de Alexa
caracter = client.recv(1)
estado_anterior = "DISPONIBLE"

#Se cambia a modo no bloqueante. De no hacer esto, el programa se quedaria parado hasta que Alexa cambiara de estado
server.setblocking(0)
client.setblocking(0)

#Servos

pwm = Adafruit_PCA9685.PCA9685() #Se inicializa el PCA9685 utilizando la direccion por defecto (0x40)
pwm.set_pwm_freq(50) #50 Hz
ctrl_mandibula = [time.time(), time.time(), False]
ctrl_orejas = [time.time(), time.time(), False]
t_parpadeo = time.time()
t_bostezo = time.time()
duracion_bostezo = 0
bostezando = False

#PTU

serial_port = serial.Serial(port = "/dev/ttyUSB0")
if(serial_port.isOpen() == False):
	serial_port.open()
pan_angle = 0.0
tilt_angle = 6.5

#Caras

face_cascade = cv2.CascadeClassifier('/home/pi/haarcascade_frontalface_default.xml') #Detector de caras
w_cara = 0
w_cara_anterior = 0
haycaras = False
caranueva = False
t_ultima_cara = time.time()

#Configuracion de la camara

#Resolucion
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

#Seleccion de la camara y configuracion de resolucion
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


####### DEFINICIONES DE FUNCIONES #######

#Funciones para el control de los servos

def oreja_izda(tanto_por_uno):
	ticks = int(round(tanto_por_uno*135 + 230))
	pwm.set_pwm(0, 0, ticks)
	
def oreja_dcha(tanto_por_uno):
	ticks = int(round((1 - tanto_por_uno)*185 + 105))
	pwm.set_pwm(1, 0, ticks)
	
def labio_sup(tanto_por_uno):
	ticks = int(round((1 - tanto_por_uno)*390 + 90))
	pwm.set_pwm(2, 0, ticks)
	
def ceja_dcha(tanto_por_uno):
	ticks = int(round((1 - tanto_por_uno)*175 + 190))
	pwm.set_pwm(3, 0, ticks)
	
def ceja_izda(tanto_por_uno):
	ticks = int(round(tanto_por_uno*135 + 230))
	pwm.set_pwm(4, 0, ticks)
	
def parpados(tanto_por_uno):
	ticks = int(round(tanto_por_uno*165 + 200))
	pwm.set_pwm(5, 0, ticks)
	
def mandibula(tanto_por_uno):
	ticks = int(round((1 - tanto_por_uno)*140 + 330))
	pwm.set_pwm(6, 0, ticks)
	
def labio_inf(tanto_por_uno):
	ticks = int(round((1 - tanto_por_uno)*415 + 70))
	pwm.set_pwm(7, 0, ticks)

def hablar(ctrl_mandibula):
	t1 = ctrl_mandibula[0]
	t2 = ctrl_mandibula[1]
	abriendo = ctrl_mandibula[2]
	if(time.time() - t2 > 0.3 and abriendo == False):
		mandibula(1)
		t1 = time.time()
		abriendo = True
	elif(time.time() - t1 > 0.3 and abriendo == True):
		mandibula(0)
		t2 = time.time()
		abriendo = False
	ctrl_mandibula = [t1, t2, abriendo]
	return ctrl_mandibula

def mover_orejas(ctrl_orejas):
	t1 = ctrl_orejas[0]
	t2 = ctrl_orejas[1]
	bajando = ctrl_orejas[2]
	if(time.time() - t2 > 0.4 and bajando == False):
		oreja_dcha(0)
		oreja_izda(0)
		t1 = time.time()
		bajando = True
	elif(time.time() - t1 > 0.4 and bajando == True):
		oreja_dcha(1)
		oreja_izda(1)
		t2 = time.time()
		bajando = False
	ctrl_orejas = [t1, t2, bajando]
	return ctrl_orejas

#Funciones para el control de la PTU

def PTU_write(pseudocomando):
	
	if(pseudocomando[0:2] == "PP" or pseudocomando[0:2] == "pp" or pseudocomando[0:2] == "TP" or pseudocomando[0:2] == "tp"):
		grados = float(pseudocomando[2:])
		if(pseudocomando[0:2] == "PP" or pseudocomando[0:2] == "pp"):
			#Se comprueba que el angulo esta dentro de los limites y si no se toma el angulo limite
			if(grados < - 159.0):
				grados = - 159.0
			elif(grados > 159.0):
				grados = 159.0
			#Se traduce el angulo a la escala que utiliza la PTU
			posicion = int(round(3095.0/159.0*grados))
		elif(pseudocomando[0:2] == "TP" or pseudocomando[0:2] == "tp"):
			#Se comprueba que el angulo esta dentro de los limites y si no se toma el angulo limite. Los limites en este caso son mas restrictivos que los limites de fabrica para asegurar que la PTU pueda con el peso de la cabeza
			if(grados < - 1.0):
				grados = - 1.0
			elif(grados > 14.0):
				grados = 14.0
			#Se traduce el angulo a la escala que utiliza la PTU
			posicion = int(round(904.0/47.0*grados))
		comando = pseudocomando[0:2] + str(posicion) + ' '
	else:
		comando = pseudocomando + ' '

	for c in range(len(comando)):
		serial_port.write(comando[c]) #Se envia el comando caracter por caracter.
		serial_port.read(1) #Los caracteres enviados son recibidos de vuelta. Es necesario leerlos
	
	#Lectura de la respuesta
	while(True):
		c = serial_port.read(1)
		if(c == '\n'):
			break

def PTU_read(pseudocomando):
	
	comando = pseudocomando + ' '

	for c in range(len(comando)):
		serial_port.write(comando[c])
		serial_port.read(1)
	
	#Lectura de la respuesta
	respuesta = ''
	while(True):
		c = serial_port.read(1)
		if(c == '\n'):
			break
		respuesta = respuesta + c
	
	if(pseudocomando == "PP" or pseudocomando == "pp" or pseudocomando == "TP" or pseudocomando == "tp"):
		if(pseudocomando == "PP" or pseudocomando == "pp"):
			try:
				posicion = int(respuesta[26:])
				grados = 159.0/3095.0*float(posicion)
				return grados
			except:
				print respuesta
				return 0.0
		elif(pseudocomando == "TP" or pseudocomando == "tp"):
			try:
				posicion = int(respuesta[27:])
				grados = 47.0/904.0*float(posicion)
				return grados
			except:
				print respuesta
				return 0.0
	else:
		print respuesta
		return 0.0

#Se posiciona la PTU en la posicion por defecto
posicionada = False
PTU_write("PP" + str(0.0))
PTU_write("TP" + str(6.5))


#Instrucciones de uso
print "INSTRUCCIONES DE USO"
print "Tecla T: hacer que el robot escuche (tambien puedes decir \"Alexa\")"
print "Tecla M: dormir y despertar al robot"
print "Tecla S: detener reproduccion multimedia"
print "Tecla 1: reanudar reproduccion multimedia"
print "Tecla Q: finalizar el programa"
print "Tecla X: finalizar el programa y apagar la Raspberry Pi"


################## BUCLE ##################

while(True):
	
	#En caso de que haya habido un cambio de estado de Alexa, se obtiene el caracter correspondiente
	try:
		caracter = client.recv(1)
	except:
		pass
	
	#Se determina el estado de Quixote
	if(caracter == 'm'):
		estado = "DURMIENDO"
	elif(caracter == 'i'):
		#Si venimos del estado "ESCUCHANDO" pero no se ha escuchado nada, vamos al estado "CONFUSO"
		if((estado_anterior == "ESCUCHANDO" or estado_anterior == "CONFUSO") and time.time() - t_fin_escucha < 2):
			estado = "CONFUSO"
		#En caso contrario, vamos al estado "DISPONIBLE"
		else:
			estado = "DISPONIBLE"
	elif(caracter == 'a'):
		estado = "ACTIVADO"
	elif(caracter == 'l'):
		estado = "ESCUCHANDO"
	elif(caracter == 't'):
		estado = "PENSANDO"
	elif(caracter == 's'):
		estado = "HABLANDO"
	#Si se recibe un caracter nulo por el socket, significa que Alexa se ha apagado
	else:
		estado = "APAGADO"
	
	#Para cada estado, se mueven los servos de una forma determinada
	if(estado == "DURMIENDO"):
		oreja_dcha(0)
		oreja_izda(0)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		parpados(0)
		labio_sup(0)
		labio_inf(0)
		mandibula(0)
	elif(estado == "CONFUSO"):
		ctrl_orejas = mover_orejas(ctrl_orejas)
		ceja_dcha(0)
		ceja_izda(0)
		#Cada cierto tiempo aleatorio, Quixote parpadea
		if(time.time() - t_parpadeo > random.uniform(4, 12)):
			parpados(0)
			t_parpadeo = time.time()
		else:
			parpados(1)
		labio_sup(0)
		labio_inf(0)
		mandibula(0.5)
	elif(estado == "DISPONIBLE"):
		if(estado_anterior != "DISPONIBLE"):
			t_bostezo = time.time() #Se reinicia el contador que se utiliza para determinar cuando bosteza Quixote
		oreja_dcha(0.5)
		oreja_izda(0.5)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		if(time.time() - t_parpadeo > random.uniform(0, 8)):
			parpados(0)
			t_parpadeo = time.time()
		else:
			parpados(0.9)
		#Si Quixote pasa mucho tiempo en estado "DISPONIBLE", bosteza
		if((time.time() - t_bostezo > random.uniform(30, 120) or bostezando) and duracion_bostezo < random.uniform(10, 30)):
			labio_sup(0)
			labio_inf(0)
			mandibula(1)
			t_bostezo = time.time()
			duracion_bostezo += 1
			bostezando = True
		else:
			labio_sup(0.5)
			labio_inf(0.5)
			mandibula(0)
			duracion_bostezo = 0
			bostezando = False
	elif(estado == "ACTIVADO"):
		oreja_dcha(0.5)
		oreja_izda(0.5)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		if(time.time() - t_parpadeo > random.uniform(4, 12)):
			parpados(0)
			t_parpadeo = time.time()
		else:
			parpados(1)
		labio_sup(1)
		labio_inf(1)
		ctrl_mandibula = hablar(ctrl_mandibula)
	elif(estado == "ESCUCHANDO"):
		oreja_dcha(1)
		oreja_izda(1)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		if(time.time() - t_parpadeo > random.uniform(4, 12)):
			parpados(0)
			t_parpadeo = time.time()
		else:
			parpados(1)
		labio_sup(1)
		labio_inf(1)
		mandibula(0)
		t_fin_escucha = time.time()
	elif(estado == "PENSANDO"):
		oreja_dcha(0.5)
		oreja_izda(0.5)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		parpados(0)
		labio_sup(0.5)
		labio_inf(0.8)
		mandibula(0)
	elif(estado == "HABLANDO"):
		oreja_dcha(0.5)
		oreja_izda(0.5)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		if(time.time() - t_parpadeo > random.uniform(4, 12)):
			parpados(0)
			t_parpadeo = time.time()
		else:
			parpados(1)
		labio_sup(1)
		labio_inf(1)
		ctrl_mandibula = hablar(ctrl_mandibula)
	
	#Se obtiene el fotograma
	ret, frame = camera.read()
	
	#Se comprueba si la PTU lleva quieta un cierto tiempo (imagen estable)
	PP = PTU_read("PP")
	TP = PTU_read("TP")
	if(abs(PP - pan_angle) <= 1 and abs(TP - tilt_angle) <= 1 and not posicionada):
		posicionada = True
		t_PTU_posicionada = time.time()
	
	#Si se tiene una imagen estable y Quixote no esta "DURMIENDO", se buscan caras
	
	if(posicionada and time.time() - t_PTU_posicionada > 0.5 and estado != "DURMIENDO"):
		
		#Se pasa el fotograma a escala de grises
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		#Se buscan caras
		caras = face_cascade.detectMultiScale(gray, 1.1, 5) #1.1 es el paso de reescalado. 5 es el numero de vecinos

		x_cara = 0 #Coordenada x de la esquina superior izquierda de la cara
		y_cara = 0 #Coordenada y de la esquina superior izquierda de la cara
		w_cara = 0 #Anchura de la cara
		h_cara = 0 #Altura de la cara
		
		#Si se ha detectado al menos una cara
		
		if(len(caras) > 0):
			
			haycaras = True
			
			#Si se han detectado varias caras, se elige la mas ancha
			for (x_i, y_i, w_i, h_i) in caras:
				if(w_cara < w_i):
					x_cara = x_i
					y_cara = y_i
					w_cara = w_i
					h_cara = h_i
			
			#Se dibuja un rectangulo verde alrededor de la cara
			cv2.rectangle(frame, (x_cara, y_cara), (x_cara + w_cara, y_cara + h_cara), (0, 255, 0), 2)
			
			#A partir de las coordenadas de la esquina superior izquierda y de la anchura, se obtienen las coordenadas del centro de la cara
			x_cara = int(x_cara + (w_cara/2))
			y_cara = int(y_cara + (h_cara/2))
			
			#Se obtienen las coordenadas del centro de la cara respecto del centro del fotograma. Estas se usaran despues para posicionar adecuadamente la PTU
			dx_cara = x_cara - CAMERA_WIDTH/2
			dy_cara = y_cara - CAMERA_HEIGHT/2
			
			#Si la cara esta considerablemente lejos del centro del fotograma, se calculan los angulos a los que se debe mover la PTU. Los nuevos angulos son los angulos de la iteracion anterior mas los que acabamos de calcular.
			if(abs(dx_cara) > CAMERA_WIDTH/20):
				pan_angle = pan_angle - 180.0/math.pi*math.atan(3.0466e-3*float(dx_cara))
			if(abs(dy_cara) > CAMERA_HEIGHT/20):
				tilt_angle = tilt_angle - 180.0/math.pi*math.atan(2.8485e-3*float(dy_cara))
			
			#Se comprueba si los angulos estan dentro del angulo limite y si no tomamos el angulo limite
			if(pan_angle < - 159.0):
				pan_angle = - 159.0
			elif(pan_angle > 159.0):
				pan_angle = 159.0
			if(tilt_angle < - 1.0):
				tilt_angle = - 1.0
			elif(tilt_angle > 14.5):
				tilt_angle = 14.5
			
			#Si la diferencia entre los angulos actuales y los nuevos es significativa, se mueve la PTU
			if(abs(PP - pan_angle) > 1 or abs(TP - tilt_angle) > 1):
				posicionada = False
				PTU_write("PP" + str(pan_angle))
				PTU_write("TP" + str(tilt_angle))
			
			#Si han transcurrido mas de dos segundos entre detecciones de caras, o bien si la cara esta muy lejos del centro del fotograma, o bien si el tamano de la cara ha cambiado considerablemente, se considera que la cara es nueva
			if(time.time() - t_ultima_cara > 2 or abs(PP - pan_angle) > 10 or abs(TP - tilt_angle) > 10 or w_cara > 1.25*w_cara_anterior):
				caranueva = True
			else:
				caranueva = False
		
		#Si se han buscado caras pero no se ha detectado ninguna
		
		else:
			
			haycaras = False
			caranueva = False
	
	#Si aun no se tiene una imagen estable o Quixote esta "DURMIENDO"
	
	else:
		
		#Si el estado es "DURMIENDO", se mueve la PTU a su posicion por defecto. Si no, se mantiene donde esta
		if(estado == "DURMIENDO"):
			pan_angle = 0.0
			tilt_angle = 5.0
			posicionada = False
			PTU_write("PP" + str(pan_angle))
			PTU_write("TP" + str(tilt_angle))
			haycaras = False
			caranueva = False
	
	#Si hay una cara nueva y Quixote esta "DISPONIBLE", se pone a escuchar
	if(caranueva and estado == "DISPONIBLE"):
		client.send('t')
	
	#Variables para la siguiente iteracion
	estado_anterior = estado
	caranueva = False
	if(haycaras):
		w_cara_anterior = w_cara
		t_ultima_cara = time.time()
	
	#Visualizacion del fotograma capturado
	cv2.imshow("Frame", frame)
	
	#Comandos por teclado
	
	key = cv2.waitKey(1) & 0xFF
	
	#Tecla 'q' (o bien la aplicacion de Alexa ha sido cerrada) -> terminar el programa
	#Tecla 'x' o '4' -> terminar el programa y apagar la Raspberry Pi
	if(key == ord('q') or key == ord('x') or key == ord('4') or estado == "APAGADO"):
		#Apagar Alexa
		client.send('q')
		#Cerrar el socket
		server.close()
		#Mover los servos a su posicion inicial (Quixote durmiendo)
		oreja_dcha(0)
		oreja_izda(0)
		ceja_dcha(0.5)
		ceja_izda(0.5)
		parpados(0)
		labio_sup(0)
		labio_inf(0)
		mandibula(0)
		#Mover la PTU a su posicion inicial
		PTU_write("PP" + str(0.0))
		PTU_write("TP" + str(6.5))
		#Cerrar el puerto serie
		serial_port.close()
		#Fin del bucle
		break
		
	#Tecla 't' o '2' -> hacer que Quixote escuche
	elif(key == ord('t') or key == ord('2')):
		client.send('t')
	
	#Tecla 'm' o '3' -> apagar y encender el microfono
	elif((key == ord('m') or key == ord('3')) and (estado == "DISPONIBLE" or estado == "DURMIENDO")): #El microfono no se apagara si el estado de Quixote no es "DISPONIBLE" o "DURMIENDO", pues de otro modo aparecen problemas a la hora de determinar el estado de Quixote
		client.send('m')
	
	#Tecla 's' o '0' -> detener reproduccion multimedia
	elif(key == ord('s') or key == ord('0')):
		client.send('s')
	
	#Tecla '1' -> reanudar reproduccion multimedia
	elif(key == ord('1')):
		client.send('1')
		
#Apagar Raspberry Pi
if(key == ord('x') or key == ord('4')):
	subprocess.call("sudo poweroff", shell=True)
