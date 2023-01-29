

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

nodes = list(range(0, number_nodes))
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


m = pyo.ConcreteModel()

m.nodes = pyo.RangeSet(0,number_nodes-1)
m.nodes2 = pyo.RangeSet(1,number_nodes)
m.nodes3 = pyo.RangeSet(1,number_nodes-1)
m.nodes4 = pyo.RangeSet(0,number_nodes)
m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)



m.x = pyo.Var(m.nodes4,m.nodes4, domain=pyo.Binary)
m.y = pyo.Var(m.nodes4,domain=pyo.NonNegativeReals)
m.z = pyo.Var(m.nodes,m.drones, domain=pyo.Binary)

m.makespan = pyo.Var(domain=pyo.NonNegativeReals)



# Constraint 1:
m.orders_fulfillment = pyo.ConstraintList()
for i in m.nodes3:
    # m.orders_fulfillment.add(sum(m.x[i,j] for j in m.nodes2) + sum(m.z[i,d] for d in m.drones) == 1)
    m.orders_fulfillment.add(sum(m.x[i, j] for j in m.nodes2) + sum(m.z[i, d] for d in m.drones) >= 1)

#constraint 2:
m.flows = pyo.ConstraintList()
for h in m.nodes3:
    m.flows.add(sum(m.x[i,h] for i in m.nodes if i != h) - sum(m.x[h,j] for j in m.nodes2 if j != h) == 0)

# constraint 3:
m.max_routes = pyo.ConstraintList()
m.max_routes.add(sum(m.x[0,j] for j in m.nodes3) <= number_workers)

# constraint 4
m.subroutes = pyo.ConstraintList()
for i in m.nodes4:
    for j in m.nodes4:
        m.subroutes.add(m.y[i] + m.x[i,j] - worker_capacity*(1-m.x[i,j]) <= m.y[j])

# constraint 5
m.capacity = pyo.ConstraintList()
for i in m.nodes2:
    m.capacity.add(m.y[i] <= worker_capacity)
    m.capacity.add(1<= m.y[i])

# constraint 6
# m.return_depot = pyo.ConstraintList()
# m.return_depot.add(sum(m.x[j,6] for j in m.nodes3) == number_workers)



# Makespan workers
m.makespan_workers = pyo.ConstraintList()
m.makespan_workers.add(sum(m.x[i, j]*worker_distances[i,j] for i in m.nodes4 for j in m.nodes4) <= m.makespan)

# makespan drones
m.makespan_drones = pyo.ConstraintList()
for d in m.drones:
        m.makespan_drones.add(sum(m.z[i, d] * drone_distances[0, i-1] * 2 for i in m.nodes3) <= m.makespan)



m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)


solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

for i in range(len(list(m.x[0,:].value))):
    print(i)
    print(list(m.x[i, :].value))

# # print(list(m.y[:].value))
print('\n')
print(list(m.z[:,:].value))
# print(sum(list(m.x[:,:].value)))


print(m.makespan.value)











