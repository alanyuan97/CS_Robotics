
import numpy as np

x = [-1,-1.2,-0.1,-1.2,2.7,2.1,3.1,2.4,0.3,1.3]
y = [-4.3,0.6,-0.1,-0.1,-0.8,-1.7,-2.2,-1.4,-1.4,-4.7]
cov_mat = np.stack((x,y),axis=0)
print("Shape: \n", np.shape(cov_mat))
print("Input/Output in matrix form: \n",cov_mat)
print("Covariance of (X,Y): \n", np.cov(cov_mat))
sumnumx = sum(x)/10
sumnumy = sum(y)/10
print("Average point: ", sumnumx," ",sumnumy)

