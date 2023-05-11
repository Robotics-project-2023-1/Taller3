import numpy as np

alphas = [0, 90, 0]
ds = [10, 0 ,0] # Altura en cm a primer eslabo, 0, 0
a_s = [0, 10, 10] #0, longitud eslabon1, longitud eslabon2 en cm

angulos = [30,20,50] #Angulos recibidos de cada joint
# Definir las dos matrices
A = [[0],[0],[0]] #matrices
for i in range(3):
    a = a_s[i]
    d = ds[i]
    alpha = alphas[i]
    ang = angulos[i]
    A[i] = np.array([[np.cos(ang), -np.sin(ang)*np.cos(alpha), np.sin(ang)*np.sin(alpha), a*np.cos(ang)],
                   [np.sin(ang), np.cos(ang)*np.cos(alpha), -np.cos(ang)*np.sin(alpha), a*np.sin(ang)],
                   [0, np.sin(alpha), np.cos(alpha), d], 
                   [0, 0 , 0, 1]])

# Calcular el producto 
Acumulado = np.dot(A[0],A[1])
Acumulado = np.dot(Acumulado, A[2])

# Imprimir el resultado
print(Acumulado)
