



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



# # Makespan workers
# m.makespan_workers = pyo.ConstraintList()
# m.makespan_workers.add(sum(m.x_workers[i, j]*worker_distances[i,j] for i in m.nodes for j in m.nodes) <= m.makespan)
#
# # makespan drones: have to consider the distances of the routes with v
# m.makespan_drones = pyo.ConstraintList()
# m.makespan_workers.add(sum(m.x_drones[i, j,r]*drone_distances[i,j]  for i in m.nodes for j in m.nodes for r in m.trips)
#                        + sum(m.x_drones[i,j,r]*m.v[i,k,r]*(drone_distances[i,k]+drone_distances[k,j]-drone_distances[i,j])
#                                                          for i in m.nodes for j in m.nodes for r in m.trips for k in m.nodes)
#                            <= m.makespan)