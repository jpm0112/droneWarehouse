

import pyomo.environ as pyo
import numpy as np
import matplotlib.pyplot as plt
from datetime import *


np.random.seed(1049586)
n = 6
rnd = np.random
rnd.seed(12)
coord_x = [0]
coord_y = [0]
for i in range(n - 1):
    coord = rnd.rand() * 100
    coord_x.append(coord)

for i in range(n - 1):
    coord = rnd.rand() * 100
    coord_y.append(coord)

coord_x[-2] = 200
coord_y[-2] = 200
coord_x[-3] = 210
coord_y[-3] = 180
depot = [0]
clientes = [i for i in range(1,n)] #[] es lista
nodes = depot + clientes# suma de lista concatena las dos listas

coord_x[-1] = coord_x[0]
coord_y[-1] = coord_y[0]



drone_distances = np.random.randint(1,10,size=(n,n))
for i in range(n):
    for j in range(n):
        point1 = np.array((coord_x[i],coord_y[i]))
        point2 = np.array((coord_x[j], coord_y[j]))
        drone_distances[i,j] = np.linalg.norm(point1 - point2)

# WORKER DISTANCE MATRIX
worker_distances = drone_distances*4                                                #CHANGE FOR WORKER DISTANCE MATRIX
np.fill_diagonal(drone_distances,1000)
np.fill_diagonal(worker_distances,1000)

number_workers = 1
number_drones = 1
number_nodes = len(drone_distances)-1
worker_capacity = 10000
max_time = 10000000
drone_capacity = 1
drone_capacity = drone_capacity +1
big_M = 1000000

demand = np.ones(number_nodes)
demand[0] = 0
demand[-1] = 0
demand = demand.astype("int")

nodes_list = list(range(0, number_nodes))
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


m = pyo.ConcreteModel()

m.nodes1 = pyo.RangeSet(0, number_nodes-1)
m.nodes2 = pyo.RangeSet(1,number_nodes)
m.orders = pyo.RangeSet(1, number_nodes-1) #orders
m.nodes = pyo.RangeSet(0, number_nodes) #total nodes
m.trips = pyo.RangeSet(0, number_nodes-1)

m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)

# worker variable
m.x_workers = pyo.Var(m.nodes, m.nodes, domain=pyo.Binary)
m.y_workers = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

#dorne variable
m.x_drones = pyo.Var(m.nodes, m.nodes,m.trips, domain=pyo.Binary)
m.y_drones = pyo.Var(m.nodes,m.trips, domain=pyo.NonNegativeReals)

# drop off variables
m.v = pyo.Var(m.nodes, m.nodes,m.trips, domain=pyo.Binary)
m.t = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

m.makespan = pyo.Var(domain=pyo.NonNegativeReals)



# Constraint 1:
m.orders_fulfillment = pyo.ConstraintList()
for i in m.orders:
    m.orders_fulfillment.add(sum(m.x_workers[i, j] for j in m.nodes2) + sum(m.x_drones[i, j, r] for j in m.nodes2 for r in m.trips) >= 1)

#constraint 2:
m.flows = pyo.ConstraintList()
for h in m.orders:
    m.flows.add(sum(m.x_workers[i,h] for i in m.nodes1 if i != h) - sum(m.x_workers[h,j] for j in m.nodes2 if j != h) == 0)
for h in m.orders:
    for r in m.trips:
        m.flows.add(sum(m.x_drones[i, h, r] for i in m.nodes1 if i != h) - sum(m.x_drones[h, j, r] for j in m.nodes2 if j != h) == 0)


# constraint 3:
m.max_routes = pyo.ConstraintList()
m.max_routes.add(sum(m.x_workers[0,j] for j in m.orders) <= number_workers)
for r in m.trips:
    m.max_routes.add(sum(m.x_drones[0,j,r] for j in m.orders) <= 1)
# m.max_routes.add(sum(m.x_drones[0,j] for j in m.orders) <= number_drones) #seems this one is not needed

# constraint 4
m.subroutes = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.subroutes.add(m.y_workers[i] + m.x_workers[i,j] - worker_capacity * (1 - m.x_workers[i,j]) <= m.y_workers[j]) #todo ADD drone load if drone drops

for i in m.nodes:
    for j in m.nodes:
        for k in m.nodes:
            for r in m.trips:
                 m.subroutes.add(m.y_drones[i,r]
                                 + m.x_drones[i, j, r] * demand[j]
                                  - m.v[i, k, r] * (drone_capacity)
                                 - drone_capacity * (1 - m.x_drones[i, j, r])
                                 <= m.y_drones[j,r])  # todo include order's required capacity (it's counting node cero's demand as 1)
                # m.subroutes.add(m.y_drones[i, r]
                #                 + m.x_drones[i,j,r] * 1
                #                 - m.v[i,k,r]*(1 +  m.y_drones[i, r])
                #                 - drone_capacity * (1 - m.x_drones[i, j, r])
                #                 <= m.y_drones[j, r])  # todo include order's required capacity (it's counting node cero's demand as 1)
                # m.subroutes.add(sum(m.x_drones[i,j] for i in m.nodes for j in m.nodes) <=3)
                # m.subroutes.add((m.v[i, j] ) == 0)


# constraint 5
m.time_accumulation = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.time_accumulation.add(m.t[i] + worker_distances[i,j] * m.x_workers[i,j] - max_time * (1 - m.x_workers[i,j]) <= m.t[j])
for i in m.nodes:
    for j in m.nodes:
        for k in m.nodes:
            for r in m.trips:
                m.time_accumulation.add(
                    m.t[i]
                    + drone_distances[i, j] * m.x_drones[i, j, r]
                    - m.v[i, k, r] * (drone_distances[i, j] * m.x_drones[i, j, r])
                    + m.v[i, k, r] * (drone_distances[i, k] + drone_distances[k, j])
                    - max_time * (1 - m.x_drones[i, j, r]) <= m.t[j])



m.capacity = pyo.ConstraintList()
for i in m.nodes2:
    m.capacity.add(m.y_workers[i] <= worker_capacity)
    m.capacity.add(1 <= m.y_workers[i]) #todo one should be demand

for i in m.nodes2:
    for r in m.trips:
        m.capacity.add(m.y_drones[i,r] <= drone_capacity)
        m.capacity.add(1 <= m.y_drones[i,r])

m.no_unloading_normal_route = pyo.ConstraintList()
for i in m.orders:
    for j in m.orders:
        for r in m.trips:
            m.no_unloading_normal_route.add(m.x_drones[i,j,r] <= (1-m.v[i,j,r]))



m.drone_drops = pyo.ConstraintList()
for j in m.orders:
        m.drone_drops.add(sum(m.x_workers[h,j] for h in m.orders) >= m.v[i,j,r])
m.drone_drops.add(sum(m.v[0,j,r] for j in m.nodes for r in m.trips )==0)
m.drone_drops.add(sum(m.v[i,0,r] for i in m.nodes for r in m.trips )==0)


# drone cannot drop load from node j if the drone doesnt travel to it in the same trip
# for r in m.trips:
#     for j in m.nodes:
#         m.drone_drops.add(sum(m.x_drones[i,j,r] for i in m.nodes) >= sum(m.v[j,k,r] for k in m.nodes))


for i in m.nodes:
    for r in m.trips:
        m.drone_drops.add(sum(m.v[i, j, r] for j in m.nodes) <= 1)

m.drop_time_coordination = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        for r in m.trips:
            m.drop_time_coordination.add(m.t[j] - m.t[i] - drone_distances[i, j] <= big_M * (1 - m.v[i, j, r]))



# Makespan workers
m.makespan_workers = pyo.ConstraintList()
m.makespan_workers.add(sum(m.x_workers[i, j]*worker_distances[i,j] for i in m.nodes for j in m.nodes) <= m.makespan)

# makespan drones
m.makespan_drones = pyo.ConstraintList()
m.makespan_workers.add(sum(m.x_drones[i, j,r]*drone_distances[i,j]  for i in m.nodes for j in m.nodes for r in m.trips)
                       + sum(m.x_drones[i,j,r]*m.v[i,k,r]*(drone_distances[i,k]+drone_distances[k,j]-drone_distances[i,j])
                                                         for i in m.nodes for j in m.nodes for r in m.trips for k in m.nodes)
                           <= m.makespan)

m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)

start_time = datetime.now()
solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

end_time = datetime.now()
print('Duration: {}'.format(end_time - start_time))

print('WORKERS')
for i in range(len(list(m.x_workers[0, :].value))):
    print(i)
    print(list(m.x_workers[i, :].value))
print('DRONEs')
for r in range(number_nodes):
    for i in range(number_nodes):
        print(i,r)
        print(list(m.x_drones[i, :, r].value))

print('DRONEs drop')
for r in range(number_nodes):
    for i in range(number_nodes):
        for j in range(number_nodes):

            if m.v[i, j, r].value == 1:

                    print(i, j, r)
                    print((m.v[i, j, r].value))

print("\n")




print(print(list(m.y_drones[:, 0].value)))
# print('DRONEs drop')
# for i in range(len(list(m.v[0, :, :].value))):
#     print(i)
#     print(list(m.v[i, :,:].value))

print('times: ',list(m.t[:].value))
print("makespan: ",m.makespan.value)

routes_worker = []
current_node = 0
while 1 in list(m.x_workers[current_node, :].value):
    next_node = list(m.x_workers[current_node, :].value).index(1)
    routes_worker.append([current_node,next_node])
    current_node =next_node

routes_drone = []
for r in range(number_nodes):
    route_drone = []
    current_node = 0

    while 1 in [round(num, 0) for num in list(m.x_drones[current_node, :,r].value)]:
        next_node = [round(num, 0) for num in list(m.x_drones[current_node, :,r].value)].index(1)
        # if 1 in [round(num, 0) for num in list(m.v[current_node, :,r].value)]:
        #     next_node_drop = [round(num, 0) for num in list(m.v[current_node, :,r].value)].index(1)
        #     route_drone.append([current_node, next_node_drop])
        #     current_node = next_node_drop

        route_drone.append([current_node, next_node])
        current_node = next_node
    if len(route_drone)>0:
        routes_drone.append(route_drone)


print('DRONEs drop')
for r in range(number_nodes):
    for i in range(number_nodes):
        for j in range(number_nodes):

            if m.v[i, j, r].value == 1:
                    print(i, j, r)
print(routes_worker)
print(routes_drone)





plt.scatter(coord_x,coord_y,c='r')
counter = 0
for i,j in routes_worker:
        plt.plot([coord_x[i], coord_x[j]], [coord_y[i], coord_y[j]],c='b', zorder=0,alpha=0.4, label = "Worker")
        if i != nodes[-1] and counter == 1:
            plt.annotate(" "+str(i)+"*"+'time: '+str(round(m.t[i].value)), (coord_x[i], coord_y[i]))
        else:
            plt.annotate( " "+str(i)+'time: '+str(round(m.t[i].value)), (coord_x[i], coord_y[i]))
        counter = counter + 1


for z in range(len(routes_drone)):
    route_drone = routes_drone[z]
    counter = 0
    for i, j in route_drone:
        if any(x ==1 for x in list(m.v[i,:,z].value)) == 1:
            unload_node = [t for t in range(len(list(m.v[i,:,z].value))) if list(m.v[i,:,z].value)[t] == 1][0]
            plt.plot([coord_x[i], coord_x[unload_node]], [coord_y[i], coord_y[unload_node]], c='r', zorder=0, alpha=1, label='Drone')
            plt.plot([coord_x[unload_node], coord_x[j]], [coord_y[unload_node], coord_y[j]], c='r', zorder=0, alpha=1, label='Drone')
        else:
            plt.plot([coord_x[i], coord_x[j]], [coord_y[i], coord_y[j]], c='g', zorder=0, alpha=0.4, label='Drone')

        if counter == 1:
            plt.annotate(" "+str(i)+"*"+'time: '+str(round(m.t[i].value)), (coord_x[i], coord_y[i]))
        elif i != nodes[-1]:
            plt.annotate( " "+str(i)+'time: '+str(round(m.t[i].value)), (coord_x[i], coord_y[i]))
        counter = counter + 1
plt.show()






