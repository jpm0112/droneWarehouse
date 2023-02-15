



import numpy as np
import matplotlib.pyplot as plt

mean = [0,0]
cov = [[1,-0.5],[-0.5,1]] #identity matrix

sample = np.random.multivariate_normal(mean, cov, 100)

x, y = sample.T
plt.scatter(x, y)
plt.xlabel('x1')
plt.ylabel('x2')
plt.title('Gaussian Scatterplot with -0.5s covariance') #change tittle

#plt.savefig('gaussian_scatterplot.pdf', format='pdf') #Uncomment to save pdf
plt.show()