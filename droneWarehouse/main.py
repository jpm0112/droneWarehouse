

import pyomo.environ as pyo
import numpy as np

data_type = "float"
drone_distances = np.genfromtxt("drone_distances.csv",delimiter=',').astype(data_type)
worker_distances = np.genfromtxt("worker_distances.csv",delimiter=',').astype(data_type)

# drone_distances = 1000*(np.diag(np.ones(8)))
worker_distances = 1000 * (np.diag(np.ones(8)))


drone_distances[0,0] = 1000
worker_distances[0,0] = 1000

number_workers = 1
number_drones = 1
number_nodes = len(drone_distances)
number_orders = 5

nodes = list(range(0, number_nodes))
orders = list(np.random.choice(number_nodes,number_orders, replace= False))
orders = orders.append(0)
workers = list(range(0, number_workers))
drones = list(range(0, number_drones))


m = pyo.ConcreteModel()

m.nodes = pyo.Set(initialize=nodes)
m.nodes2 = pyo.RangeSet(1,number_nodes+1)
m.orders = pyo.Set(initialize=orders)
m.workers = pyo.Set(initialize=workers)
m.drones = pyo.Set(initialize=drones)



m.x = pyo.Var(m.nodes,m.nodes, domain=pyo.Binary)
m.y = pyo.Var(m.nodes,m.drones, domain=pyo.Binary)
m.makespan = pyo.Var(domain=pyo.NonNegativeReals)



# Constraint 1:
m.orders_fulfillment = pyo.ConstraintList()
for j in m.nodes:
    # m.orders_fulfillment.add(sum(m.y[j,d] for d in m.drones) + sum(m.x[i,j,k] for i in m.nodes if j!=i for k in m.workers) == 1   )
    # m.orders_fulfillment.add(sum(m.y[j, d] for d in m.drones) + sum(m.x[j, i, k] for i in m.nodes if j!=i for k in m.workers) == 1)
    m.orders_fulfillment.add(
        sum( m.x[i, j] for i in m.nodes if j!=i) == 1)
    m.orders_fulfillment.add(
        sum(m.x[j, i] for i in m.nodes if j!=i) == 1)


# subroutes delete
m.subroutes = pyo.ConstraintList()
for i in list(filter(lambda num: num != 0, m.nodes)):
    for j in list(filter(lambda num: num != 0 and num != i, m.nodes)):
            m.subroutes.add(m.u[i]-m.u[j] + number_nodes*m.x[i,j] <= number_nodes-1)


m.flows = pyo.ConstraintList()

for h in m.nodes:
    m.flows.add(sum(m.x[i,h] for i in m.nodes) - sum())




#Makespan workers
m.makespan_workers = pyo.ConstraintList()
for k in m.workers:
    m.makespan_workers.add(sum(m.x[i,j]*worker_distances[i,j] for i in m.nodes for j in m.nodes) <= m.makespan)

#makespan drones
m.makespan_drones = pyo.ConstraintList()
for d in m.drones:
    m.makespan_drones.add(sum(m.y[i,d] * drone_distances[0,i] * 2  for i in m.nodes ) <= m.makespan)

m.OBJ = pyo.Objective(expr=m.makespan, sense=pyo.minimize)



solver = "gurobi"
opt = pyo.SolverFactory(solver)
opt.solve(m)

print(list(m.x[:,:].value))
print(list(m.y[:,:].value))
print(list(m.u[:].value))

sum(list(m.x[:,:].value))


print(m.makespan.value)
















