

import pyomo.environ as pyo
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


# GENERATE NODES AND DISTANCES
np.random.seed(1049586)
n = 8 # with 6 is good instance, 8 is the one that i want to show
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

coord_x[3] = coord_x[3] + 10
coord_y[3] = coord_y[3] + 10
depot = [0]
orders_list = [i for i in range(1, n)] #[] es lista
nodes = depot + orders_list# suma de lista concatena las dos listas

coord_x[-1] = coord_x[0]
coord_y[-1] = coord_y[0]

#drone distances
drone_distances = np.random.randint(1,10,size=(n,n))
for i in range(n):
    for j in range(n):
        point1 = np.array((coord_x[i],coord_y[i]))
        point2 = np.array((coord_x[j], coord_y[j]))
        drone_distances[i,j] = np.linalg.norm(point1 - point2)

# WORKER DISTANCE MATRIX
worker_distances = drone_distances*4                        #CHANGE FOR WORKER DISTANCE MATRIX
np.fill_diagonal(drone_distances,1000000)
np.fill_diagonal(worker_distances,1000000)





# DEFINE PARAMETERS
number_workers = 1
number_drones = 1
number_nodes = len(drone_distances)-1
worker_capacity = 10000
max_time = 1000000000
drone_capacity = 1                                  #DRONE CAPACITY
drone_range = 700
big_M = 1000000
max_time = big_M

nodes_list = list(range(0, number_nodes))
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


demand = np.ones(number_nodes+1)
demand[0] = 0
demand[-1] = 0
demand = demand.astype("int")


m = pyo.ConcreteModel()

# DEFINE SETS
m.nodes1 = pyo.RangeSet(0, number_nodes-1)
m.nodes2 = pyo.RangeSet(1,number_nodes)
m.orders = pyo.RangeSet(1, number_nodes-1) #orders
m.nodes = pyo.RangeSet(0, number_nodes) #total nodes
m.trips = pyo.RangeSet(0, number_nodes)

m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)

#DEFINE VARIABLES
# worker variable
m.x_workers = pyo.Var(m.nodes, m.nodes, domain=pyo.Binary)
m.y_workers = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

#dorne variable
m.x_drones = pyo.Var(m.nodes, m.nodes,m.trips, domain=pyo.Binary)
m.y_drones = pyo.Var(m.nodes,m.trips, domain=pyo.NonNegativeReals) #acumulated capacity of the drone at node i in trip r

# drop off variables
m.v = pyo.Var(m.nodes, m.nodes,m.trips, domain=pyo.Binary) #if drone drops from node i to drone j in trip r
m.w = pyo.Var(m.nodes, m.nodes,m.trips, domain=pyo.Binary) # if drone waits for worker when dropping (0 if worker waits for drone)
m.t = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

m.makespan = pyo.Var(domain=pyo.NonNegativeReals)


# CONSTRAINTS

# Constraint 1: NODES MUST BE VISITED
m.orders_fulfillment = pyo.ConstraintList()
for i in m.orders:
    m.orders_fulfillment.add(sum(m.x_workers[i, j] for j in m.nodes2) + sum(m.x_drones[i, j, r] for j in m.nodes2 for r in m.trips) >= 1)

#constraint 2: FLOWS
m.flows = pyo.ConstraintList()
for h in m.orders:
    m.flows.add(sum(m.x_workers[i,h] for i in m.nodes1 if i != h) - sum(m.x_workers[h,j] for j in m.nodes2 if j != h) == 0)
for h in m.orders:
    for r in m.trips:
        m.flows.add(sum(m.x_drones[i, h, r] for i in m.nodes1 if i != h) - sum(m.x_drones[h, j, r] for j in m.nodes2 if j != h) == 0)


# constraint 3: MAX ROUTES LEAVING THE DEPOT
m.max_routes = pyo.ConstraintList()
m.max_routes.add(sum(m.x_workers[0,j] for j in m.orders) <= number_workers)
for r in m.trips:
    m.max_routes.add(sum(m.x_drones[0,j,r] for j in m.orders) <= 1)

# constraint 4: SUBROUTE ELIMINATION AND CAPACITY UPDATE
# capacity of workers:
m.subroutes = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.subroutes.add(m.y_workers[i] + m.x_workers[i,j] - worker_capacity * (1 - m.x_workers[i,j]) <= m.y_workers[j]) #todo ADD drone load if drone drops

#delete routes thast goes from depot to depot
for r in m.trips:
    m.subroutes.add(m.x_drones[0,n-1,r] == 0)

# for drones the capacity is updated when the variable v (drop at another node) is active
for i in m.nodes:
    for j in m.nodes:
        #for k in m.nodes:
            for r in m.trips:
                 m.subroutes.add(m.y_drones[i,r]
                                 + m.x_drones[i, j, r] * demand[j]
                                 - sum(m.v[i, k, r] for k in m.nodes) * (drone_capacity)
                                 - drone_capacity * (1 - m.x_drones[i, j, r])
                                 <= m.y_drones[j,r])

# constraint 5: TIME UPDATE AND COORDINATION
# same as the capacity update but saving the time
m.time_accumulation = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.time_accumulation.add(m.t[i] + worker_distances[i,j] * m.x_workers[i,j] - max_time * (1 - m.x_workers[i,j]) <= m.t[j])

print(m.x_workers[0,1].value)
print(worker_distances[1,2])
print(m.t[2].value)
print(drone_distances[4,0])
print(list(m.v[4,2,:].value))
print(list(m.w[4,2,:].value))

for i in m.nodes:
    for j in m.nodes:
        for k in m.nodes:
            for r in m.trips:
                m.time_accumulation.add(
                    m.t[i]
                    + sum( m.x_drones[q,w,e]*drone_distances[q,w] for q in m.nodes for w in m.nodes for e in range(r)) # todo this sum needs to address the Vs
                    + drone_distances[i, j] * m.x_drones[i, j, r]
                    - m.v[i, k, r] * (drone_distances[i, j] * m.x_drones[i, j, r])
                    + m.v[i, k, r] * (drone_distances[i, k] + drone_distances[k, j])
                    + m.w[i, k, r] * (m.t[k]-m.t[i]-drone_distances[i,k])
                    - max_time * (1 - m.x_drones[i, j, r]) <= m.t[j])

# constraints to coordinate drop off in terms of times
m.drop_time_coordination = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        for r in m.trips:
            # when w = 0, worker waits
            m.drop_time_coordination.add(  (1-m.w[i,j,r])     * (m.t[j] - m.t[i] - drone_distances[i, j]) <= big_M * (1 - m.v[i, j, r]))
            # when w = 1, drone waits
            m.drop_time_coordination.add(  (m.w[i,j,r]) * (m.t[i] + drone_distances[i, j] - m.t[j]) <= big_M * (1 - m.v[i, j, r]))

            # m.drop_time_coordination.add( (m.t[j] - m.t[i] - drone_distances[i, j]) <= big_M * (1 - m.v[i, j, r])) #worker waits
            # m.drop_time_coordination.add( (m.t[i] + drone_distances[i, j] - m.t[j]) <= big_M * (1 - m.v[i, j, r])) #drone waits

            # m.drop_time_coordination.add(m.w[i,j,r]<= m.v[i,j,r])



# constraint 6: max capacity (nodes2 are the nodes minus the depot)
m.capacity = pyo.ConstraintList()
for i in m.nodes2:
    m.capacity.add(m.y_workers[i] <= worker_capacity)
    m.capacity.add(1 <= m.y_workers[i]) #todo one should be demand

for i in m.nodes2:
    for r in m.trips:
        m.capacity.add(m.y_drones[i,r] <= drone_capacity)
        m.capacity.add(0 <= m.y_drones[i,r]) # todo check this because with the drop off of the drone the y variable may be 0



# drone cannot unload in its next destination (shouldt be necessary if drops can only happen in worker route)
# m.no_unloading_normal_route = pyo.ConstraintList()
# for i in m.orders:
#     for j in m.orders:
#         for r in m.trips:
#             m.no_unloading_normal_route.add(m.x_drones[i,j,r] <= (1-m.v[i,j,r]))



m.drone_drops = pyo.ConstraintList()
# drones can only drop in a node visited by the worker
for j in m.orders:
    for r in m.trips:
        m.drone_drops.add(sum(m.x_workers[h,j] for h in m.orders) >= sum(m.v[i,j,r] for i in m.orders))

# the varibale v cannot be 1 when starting or ending at depot:
m.drone_drops.add(sum(m.v[0,j,r] for j in m.nodes for r in m.trips )==0)
m.drone_drops.add(sum(m.v[i,0,r] for i in m.nodes for r in m.trips )==0)

# m.drone_drops.add(m.v[2,3,0] == 1)                                                          #TEST



#drone cannot drop load from node j if the drone doesnt travel to it in the same trip
for r in m.trips:
    for j in m.nodes:
        m.drone_drops.add(sum(m.x_drones[i,j,r] for i in m.nodes) >= sum(m.v[j,k,r] for k in m.nodes))

# drone can drop from one node at most once
# for i in m.nodes:
#     for r in m.trips:
#         m.drone_drops.add(sum(m.v[i, j, r] for j in m.nodes) <= 1)
for r in m.trips:
    for i in m.nodes:
        m.drone_drops.add(m.y_drones[i,r] <= sum(m.x_drones[j,i,r] for j in m.nodes))

#no drops in depot nodes (with the v variable)
for r in m.trips:
    for i in m.nodes:
        m.drone_drops.add(m.v[i,n-1,r] == 0)
        m.drone_drops.add(m.v[i, 0, r] == 0)


# range constraint
# m.range = pyo.ConstraintList()
# for r in m.trips:
#     m.range.add(sum(drone_distances[i, j] * m.x_drones[i, j, r]
#                     - m.v[i, k, r] * (drone_distances[i, j] * m.x_drones[i, j, r])
#                     + m.v[i, k, r] * (drone_distances[i, k] + drone_distances[k, j])
#                     + m.w[i, k, r] * (m.t[k]-m.t[i]-drone_distances[i,k]) for i in m.nodes for j in m.nodes for k in m.nodes) <= drone_range)

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

m.makespan_times = pyo.ConstraintList()
for i in m.nodes:
    m.makespan_times.add(m.t[i] <= m.makespan)

m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)



# SOLVE MODEL

start_time = datetime.now()
solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

end_time = datetime.now()
print('Duration: {}'.format(end_time - start_time))


#PRINT STUFF

print('DRONEs drop')
for r in range(number_nodes+1):
    for i in range(number_nodes+1):
        for j in range(number_nodes+1):
            if m.v[i, j, r].value == 1:
                    print(i, j, r)
                    print((m.w[i, j, r].value))
print("\n")




print('times: ',list(m.t[:].value))
print("makespan: ",m.makespan.value)

# print('DRONEs')
# print(list(m.x_drones[4, :, 5].value))


# print('capacity drone')
# for r in range(len(list(m.y_drones[:, 0].value))):
#     print(r)
#     print(list(m.y_drones[:, r].value))


# GET ROUTES
routes_worker = []
current_node = 0

while 1 in [round(num,0) for num in list(m.x_workers[current_node, :].value)]:
    next_node = [round(num,0) for num in list(m.x_workers[current_node, :].value)].index(1)
    routes_worker.append([current_node,next_node])
    current_node =next_node

routes_drone = []
for r in range(number_nodes+1):
    route_drone = []
    current_node = 0

    while 1 in [round(num, 0) for num in list(m.x_drones[current_node, :,r].value)]:
        next_node = [round(num, 0) for num in list(m.x_drones[current_node, :,r].value)].index(1)

        route_drone.append([current_node, next_node])
        current_node = next_node
    #if len(route_drone)>0:
    routes_drone.append(route_drone)


print("\n")
print("routes workers: ",routes_worker)
print("routes drones: ",routes_drone)


# PLOT STUFF
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
    for arc in route_drone:
        if any(x ==1 for x in [round(num, 0) for num in list(list(m.v[arc[0],:,z].value))]) == True:
            unload_node = [t for t in range(len(list(m.v[arc[0],:,z].value))) if [round(num, 0) for num in list(list(m.v[arc[0],:,z].value))][t] == 1][0]
            plt.plot([coord_x[arc[0]], coord_x[unload_node]], [coord_y[arc[0]], coord_y[unload_node]], c='r', zorder=0, alpha=1, label='Drone')
            plt.plot([coord_x[unload_node], coord_x[arc[1]]], [coord_y[unload_node], coord_y[arc[1]]], c='r', zorder=0, alpha=1, label='Drone')
        else:
            plt.plot([coord_x[arc[0]], coord_x[arc[1]]], [coord_y[arc[0]], coord_y[arc[1]]], c='g', zorder=0, alpha=0.4, label='Drone')

        if counter == 1:
            plt.annotate(" "+str(arc[0])+"*"+'time: '+str(round(m.t[arc[0]].value)), (coord_x[arc[0]], coord_y[arc[0]]))
        elif i != nodes[-1]:
            plt.annotate( " "+str(arc[0])+'time: '+str(round(m.t[arc[0]].value)), (coord_x[arc[0]], coord_y[arc[0]]))
        counter = counter + 1
plt.show()








print(worker_distances[0,2])













