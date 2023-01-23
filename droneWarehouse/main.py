

import pyomo.environ as pyo
import numpy as np

data_type = "float"
drone_distances = np.genfromtxt("drone_distances.csv",delimiter=',').astype(data_type)
worker_distances = np.genfromtxt("worker_distances.csv",delimiter=',').astype(data_type)

# drone_distances = 1000*(np.diag(np.ones(8)))
worker_distances = 1000 * (np.diag(np.ones(8)))


drone_distances[0,0] = 1000
worker_distances[0,0] = 1000

number_workers = 2
number_drones = 2
number_nodes = len(drone_distances)
number_orders = 5
worker_capacity = 10000

nodes = list(range(0, number_nodes))
orders = list(np.random.choice(number_nodes,number_orders, replace= False))
orders = orders.append(0)
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


m = pyo.ConcreteModel()

m.nodes = pyo.RangeSet(0,number_nodes)
m.nodes2 = pyo.RangeSet(1,number_nodes+1)
m.nodes3 = pyo.RangeSet(1,number_nodes)
m.nodes4 = pyo.RangeSet(0,number_nodes+1)
m.orders = pyo.Set(initialize=orders)
m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)



m.x = pyo.Var(m.nodes4,m.nodes4, domain=pyo.Binary)
m.y = pyo.Var(m.nodes4,domain=pyo.NonNegativeReals)
m.z = pyo.Var(m.nodes,m.drones, domain=pyo.Binary)

m.makespan = pyo.Var(domain=pyo.NonNegativeReals)



# Constraint 1: #todo add drones here
m.orders_fulfillment = pyo.ConstraintList()
for i in m.nodes:
    m.orders_fulfillment.add(sum(m.x[i,j] for j in m.nodes2) + sum(m.z[i,d] for d in m.drones) == 1)

#constraint 2:
m.flows = pyo.ConstraintList()
for h in m.nodes:
    m.flows.add(sum(m.x[i,h] for i in m.nodes if i != h) - sum(m.x[h,j] for j in m.nodes2 if j != h) == 0)

# constraint 3:
m.max_routes = pyo.ConstraintList()
m.max_routes.add(sum(m.x[0,j] for j in m.nodes3) <= number_workers)

# constraint 4
m.subroutes = pyo.ConstraintList()
for i in m.nodes2:
    for j in m.nodes2:
        m.subroutes.add(m.y[i] + m.x[i,j] - worker_capacity*(1-m.x[i,j]) <= m.y[j])

# constraint 5
m.capacity = pyo.ConstraintList()
for i in m.nodes4:
    m.capacity.add(m.y[i] <= worker_capacity)
    m.capacity.add(1<= m.y[i])

# Makespan workers
m.makespan_workers = pyo.ConstraintList()
m.makespan_workers.add(sum(m.x[i, j] for i in m.nodes4 for j in m.nodes4) <= m.makespan)

# makespan drones
m.makespan_drones = pyo.ConstraintList()
for d in m.drones:
        m.makespan_drones.add(sum(m.z[i, d] * drone_distances[0, i-1] * 2 for i in m.nodes3) <= m.makespan)



m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)


solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

print(list(m.x[:,:].value))
print(list(m.y[:].value))
print(list(m.z[:,:].value))
print(sum(list(m.x[:,:].value)))



print(m.makespan.value)











