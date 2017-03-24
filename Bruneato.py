#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import tf2_ros
import time
from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
ids = [] #lista que guarda as informacoes que o robo le

#variaveis de posicao e id do marcador
x = 0
y = 0
z = 0 
id = 0


tfl = 0
tempo = 0
tf_buffer = tf2_ros.Buffer()

#variaveis do bumper
esqf = 0
esqs = 0
dirf = 0
dirs = 0

#variaveis do laser
indice = 0
menor = 0

def recebe_laser(laser):

	global menor
	global indice

	menor = 1000

	for e in laser.ranges:
		if e < menor and e > 0.0001:
			menor = e
	
	indice = laser.ranges.index(menor)

def recebe_bump(msg):
	global esqf
	global esqs
	global dirf
	global dirs

	esqf = msg.leftFront
	esqs = msg.leftSide
	dirf = msg.rightFront
	dirs = msg.rightSide

	print("left front {} left side {} right front {} right side{}".format(msg.leftFront, msg.leftSide, msg.rightFront, msg.rightSide))


def recebe_marcador(lista_marcador):
	global x 
	global y
	global z
	global id
	global tempo
	global ids

	ids = [] #zera a lista de ids

	for marker in lista_marcador.markers:
		marker_dict = {} #dicionario que guarda as informacoes de id e posicao do marcador
		id = marker.id
		mark = "ar_marker_" + str(id)

		frame = "camera_frame" # Para NEato
		#frame = "head_camera" # Para Webcam

		print(tf_buffer.can_transform(frame, mark, rospy.Time(0)))

		trans = tf_buffer.lookup_transform(frame, mark, rospy.Time(0))
		
		# Separa as translacoes das rotacoes e converte para centimetro
		x = (trans.transform.translation.x)*100
		y = (trans.transform.translation.y)*100
		z = (trans.transform.translation.z)*100
		marker_dict[id]= (x,y,z) #adiciona as posicoes como chaves do respectivo id
		
		ids.append(marker_dict) #adiciona o dicionario a lista ids

		tempo = time.time() #guarda o tempo de execucao da acao atual

		print("id: {} x {} y {} z {} ".format(id, x,y,z))


if __name__=="__main__":
	rospy.init_node("girar")
 
      #os publishers enviam mensagens aos robo e os subscribers recebem dele
	velocidade = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	marcador = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, recebe_marcador)
	detecta_bumper = rospy.Subscriber('/bump', Bump, recebe_bump)
	detecta_laser = rospy.Subscriber('/scan', LaserScan, recebe_laser)


	tfl = tf2_ros.TransformListener(tf_buffer)


	while not rospy.is_shutdown():
     
         #os 5 ifs abaixo configuram o laser de acordo com o angulo do objeto que foi detectado

		if menor < 0.5 and menor > 0.00001:
			if indice >=315:
				roda = Twist(Vector3(-0.2, 0, 0), Vector3(0,0, -0.5))
				velocidade.publish(roda)
				continue

			if indice >=0 and indice <= 45:
				roda = Twist(Vector3(-0.2, 0, 0), Vector3(0,0, -0.5))
				velocidade.publish(roda)
				continue

			if indice >45 and indice <=180:
				roda = Twist(Vector3(0.2, 0, 0), Vector3(0,0, -0.5))
				velocidade.publish(roda)
				continue

			if indice > 180 and indice <315:
				roda = Twist(Vector3(0.2, 0, 0), Vector3(0,0, 0.5))
				velocidade.publish(roda)
				continue

            #os 3 ifs abaixo configuram o bumper

		if (esqf or esqs) == 1:
			roda = Twist(Vector3(-0.8, 0, 0), Vector3(0,0, -0.8))
			velocidade.publish(roda)
			continue

		if (dirf or dirs) == 1:
			roda = Twist(Vector3(-0.8, 0, 0), Vector3(0,0, 0.8))
			velocidade.publish(roda)
			continue

		if (dirf and esqf) == 1:
			roda = Twist(Vector3(-0.8, 0, 0), Vector3(0,0,1))
			velocidade.publish(roda)
			continue

            #abaixo ordena-se que o robo comece a procurar caso fique mais de 2 segundos sem ver um marcador
		if (time.time() - tempo) > 2:
			ids =[]
			roda = Twist(Vector3(0.02, 0, 0), Vector3(0,0, 0.1))
			velocidade.publish(roda)

            #abaixo itera-se sobre a lista ids
		for d in ids:
			
                   #o robo segue o marcador 50 ate estar a distancia de 70cm, entao para
			if id == 50: 
				#segue
				if x< -20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*0.002))
					velocidade.publish(roda)

				elif x > 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*-0.002))
					velocidade.publish(roda)

				elif x >= -20 and x <= 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, 0))
					velocidade.publish(roda)
					if z <= 70:
						roda = Twist(Vector3(0, 0, 0), Vector3(0,0, 0))
						velocidade.publish(roda)

                    #o robo segue o marcador 100 ate a distancia de 1 metro, entao vira e "foge"
			elif id ==100:

				if x< -20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*0.002))
					velocidade.publish(roda)

				elif x > 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*-0.002))
					velocidade.publish(roda)

				elif x >= -20 and x <= 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, 0))
					velocidade.publish(roda)
					if z <= 100:

						roda = Twist(Vector3(-0.1, 0, 0), Vector3(0,0, 0.7))
						velocidade.publish(roda)

                    #o robo segue o marcador 150 ate a distancia de 1 metro, entao "dorme" por 10 segundos
			elif id ==150:
				if x< -20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*0.002))
					velocidade.publish(roda)

				elif x > 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, x*-0.002))
					velocidade.publish(roda)

				elif x >= -20 and x <= 20:
					roda = Twist(Vector3(z*0.001, 0, 0), Vector3(0,0, 0))
					velocidade.publish(roda)
					if z <= 100:
						roda = Twist(Vector3(0, 0, 0), Vector3(0,0, 0))
						velocidade.publish(roda)
						rospy.sleep(10)
		
		rospy.sleep(0.1)
