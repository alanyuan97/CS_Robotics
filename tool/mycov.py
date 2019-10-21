
import numpy as np

x = [1.23,2.12,3.34,4.5]
y = [2.56,2.89,3.76,3.95]
cov_mat = np.stack((x,y),axis=0)
print("Shape: \n", np.shape(cov_mat))
print("Input/Output in matrix form: \n",cov_mat)
print("Covariance of (X,Y): \n", np.cov(cov_mat))