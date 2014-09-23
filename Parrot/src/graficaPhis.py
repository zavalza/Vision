#!/usr/bin/env python
import glob
import math
import numpy as np
import matplotlib.pyplot as plt
from StringIO import StringIO 


files = glob.glob('*.txt')
for f in files:
	print f
	if "muestras" in f:
		archivo = open(f, "r")
		lineas=archivo.readlines()
		datos = []
		for linea in lineas:
			#Los datos estan separados por tabulado
			datos = linea.split('\t')
			#print datos[0]
			#print datos [1]
			#Se dibujan los datos con azul
			plt.plot(datos[0],datos[1], 'bo', linewidth="1")
	else:
		archivo = open(f, "r")
		datos=archivo.read()
		#Los datos estan separados por nueva linea
		datos=np.genfromtxt(StringIO(datos), delimiter='\n')
		mediaPhi1=datos[0]
		mediaPhi2=datos[1]
		varianzaPhi1=datos[2]
		varianzaPhi2=datos[3]
		radio = math.sqrt((varianzaPhi1)+(varianzaPhi2))
		print radio
		#Se dibujan la media con rojo
		plt.plot(mediaPhi1,mediaPhi2, 'ro', linewidth="1")
		plt.text(mediaPhi1+radio,mediaPhi2+radio,f, fontsize=12)	
		circulo = plt.Circle((mediaPhi1,mediaPhi2),radio,color='g')
		circulo.set_alpha(.5)
		fig = plt.gcf()
		fig.gca().add_artist(circulo)
plt.title("Caracterizacion en Phis")
plt.grid()
plt.xlim(0,1)
plt.ylim(0,1)
plt.xlabel('Phi1')
plt.ylabel('Phi2')
plt.show()