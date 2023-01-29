import random
from pyomo.environ import *

# Create a model
model = ConcreteModel()

# Define sets
num_customers = 10
num_vehicles = 3
model.customers = Set(initialize=range(1,num_customers+1))
model.vehicles = Set(initialize=range(1,num_vehicles+1))

# Define parameters
model.demand = Param(model.customers, initialize=lambda model,i: random.randint(1,10))
model.distance = Param(model.customers, model.customers, initialize=lambda model,i,j: random.randint(1,10))
model.capacity = Param(model.vehicles, initialize=lambda model,k: random.randint(15,20))

# Define variables
model.x = Var(model.customers, model.customers, within=Binary)
model.route = Var(model.customers, model.vehicles, within=Binary)

# Define objective function

def objective_rule(model):
    return sum(model.distance[i, j] * model.x_workers[i, j] for i in model.customers for j in model.customers)
model.objective = Objective(rule=objective_rule, sense=minimize)

# Define constraints
def capacity_rule(model, k):
    return sum(model.demand[i]*model.route[i, k] for i in model.customers) <= model.capacity[k]
model.capacity_constraint = Constraint(model.vehicles, rule=capacity_rule)

def start_rule(model, i, k):
    return sum(model.x_workers[i, j] for j in model.customers if j != i) == model.route[i, k]
model.start_constraint = Constraint(model.customers, model.vehicles, rule=start_rule)

def end_rule(model, j, k):
    return sum(model.x_workers[i, j] for i in model.customers if i != j) == model.route[j, k]
model.end_constraint = Constraint(model.customers, model.vehicles, rule=end_rule)

def flow_rule(model, i):
    return sum(model.route[i, k] for k in model.vehicles) == 1
model.flow_constraint = Constraint(model.customers, rule=flow_rule)

# Solve the problem using Gurobi
solver = SolverFactory('gurobi')
solver.solve(model)

# Print the solution
print("Minimum Cost: ", model.objective())
for i in model.customers:
    for k in model.vehicles:
        if value(model.route[i,k]) == 1:
            print("Customer", i, "is visited by vehicle", k)
