import numpy as np
import acado
import time

T=10
NX=4
NU=2

x0 = np.zeros((1,NX))
X=np.zeros((T+1,NX))
U=np.zeros((T,NU))
Y=np.zeros((T,NX))
yN=np.zeros((1,NX))
Q = np.diag([0.5, 0.5, 1.0, 1.0])  # state cost matrix ([0.5, 0.5, 1.0, 1.0])  
#Qf = np.diag([1.0, 1.0, 0.1, 1.0])  # state cost matrix
Qf = Q


for i in range(0,T):
  Y[i,0]=0.3 * i
  Y[i,2]=1

yN[0,0]=3
yN[0,2]=1

#print('Y', Y)
#print('yN', yN)  
#print('X',X)

counter = 0

for i in range(0,10000000):
  X, U = acado.mpc(0, 1, x0,X,U,Y,yN, np.transpose(np.tile(Q,T)), Qf, 0)   
  #time.sleep(0.001)
  if counter % 1000 == 0:
    print(counter)
  counter = counter + 1 

print('U', U)
print('X', X)
#print('Y', Y)
#print('yN', yN)
#print('U', U)

