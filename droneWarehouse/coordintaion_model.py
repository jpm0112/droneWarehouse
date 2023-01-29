

import pyomo.environ as pyo
import numpy as np


np.random.seed(1049586)
drone_distances = np.random.randint(1,10,size=(5,5))
worker_distances = drone_distances*3
np.fill_diagonal(drone_distances,1000)
np.fill_diagonal(worker_distances,1000)

number_workers = 1
number_drones = 1
number_nodes = len(drone_distances)-1
worker_capacity = 10000
max_time = 10000
drone_capacity = 2

nodes_list = list(range(0, number_nodes))
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


m = pyo.ConcreteModel()

# m.nodes1 = pyo.RangeSet(0, number_nodes)
# m.nodes2 = pyo.RangeSet(1,number_nodes+1)
# m.orders = pyo.RangeSet(1, number_nodes) #orders
# m.nodes = pyo.RangeSet(0, number_nodes + 1) #total nodes

m.nodes1 = pyo.RangeSet(0, number_nodes-1)
m.nodes2 = pyo.RangeSet(1,number_nodes)
m.orders = pyo.RangeSet(1, number_nodes-1) #orders
m.nodes = pyo.RangeSet(0, number_nodes) #total nodes

m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)



m.x_workers = pyo.Var(m.nodes, m.nodes, domain=pyo.Binary)
m.y_workers = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

m.x_drones = pyo.Var(m.nodes, m.nodes, domain=pyo.Binary)
m.y_drones = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

m.z = pyo.Var(m.nodes1, m.drones, domain=pyo.Binary)
m.v = pyo.Var(m.nodes, m.nodes, domain=pyo.Binary)
m.t = pyo.Var(m.nodes, domain=pyo.NonNegativeReals)

m.makespan = pyo.Var(domain=pyo.NonNegativeReals)



# Constraint 1:
m.orders_fulfillment = pyo.ConstraintList()
for i in m.orders:
    # m.orders_fulfillment.add(sum(m.x[i,j] for j in m.nodes2) + sum(m.z[i,d] for d in m.drones) == 1)
    m.orders_fulfillment.add(sum(m.x_workers[i, j] for j in m.nodes2) + sum(m.x_drones[i, j] for j in m.nodes2) >= 1)

#constraint 2:
m.flows = pyo.ConstraintList()
for h in m.orders:
    m.flows.add(sum(m.x_workers[i,h] for i in m.nodes1 if i != h) - sum(m.x_workers[h,j] for j in m.nodes2 if j != h) == 0)
    m.flows.add(sum(m.x_drones[i, h] for i in m.nodes1 if i != h) - sum(m.x_drones[h, j] for j in m.nodes2 if j != h) == 0)

# constraint 3:
m.max_routes = pyo.ConstraintList()
m.max_routes.add(sum(m.x_workers[0,j] for j in m.orders) <= number_workers)
m.max_routes.add(sum(m.x_drones[0,j] for j in m.orders) <= number_drones)

# constraint 4
m.subroutes = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.subroutes.add(m.y_workers[i] + m.x_workers[i,j] - worker_capacity * (1 - m.x_workers[i,j]) <= m.y_workers[j])
        m.subroutes.add(m.y_drones[i] + m.x_drones[i, j] - drone_capacity * (1 - m.x_drones[i, j]) <= m.y_drones[j]) #todo include order's required capacity (it's counting node cero's demand as 1)

# constraint 5
m.time_accumulation = pyo.ConstraintList()
for i in m.nodes:
    for j in m.nodes:
        m.time_accumulation.add(m.t[i] + worker_distances[i,j] * m.x_workers[i,j] - max_time * (1 - m.x_workers[i,j]) <= m.t[j])
        m.time_accumulation.add(m.t[i] + drone_distances[i,j] * m.x_drones[i, j] - max_time * (1 - m.x_drones[i, j]) <= m.t[j])



m.capacity = pyo.ConstraintList()
for i in m.nodes2:
    m.capacity.add(m.y_workers[i] <= worker_capacity)
    m.capacity.add(1 <= m.y_workers[i])
    m.capacity.add(m.y_drones[i] <= drone_capacity)
    m.capacity.add(1 <= m.y_drones[i])



# Makespan workers
m.makespan_workers = pyo.ConstraintList()
m.makespan_workers.add(sum(m.x_workers[i, j]*worker_distances[i,j] for i in m.nodes for j in m.nodes) <= m.makespan)

# makespan drones
m.makespan_drones = pyo.ConstraintList()
for d in m.drones:
        m.makespan_workers.add(sum(m.x_drones[i, j]*drone_distances[i,j] for i in m.nodes for j in m.nodes) <= m.makespan)



m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)


solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

print('WORKERS')
for i in range(len(list(m.x_workers[0, :].value))):
    print(i)
    print(list(m.x_workers[i, :].value))
print('DRONEs')
for i in range(len(list(m.x_drones[0, :].value))):
    print(i)
    print(list(m.x_drones[i, :].value))

# print(list(m.y[:].value))
# print(list(m.z[:,:].value))
# print(sum(list(m.x_workers[:, :].value)))


print(m.makespan.value)











