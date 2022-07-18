# MPC for SeaTac controller
using JuMP
using DataFrames
using CSV
using Gurobi

A=[ 0.9518   -0.1214   -0.0328    0.3436
   -0.0013    0.8080    0.0065    0.1086
    0.0894    0.0817    0.9005   -0.0033
   -0.0011    0.0933    0.0009    0.9356];  # A matrix

   B=[2.9354   -0.8721
    -0.0867   -0.2047
     0.6692    4.7819
     0.9076    0.1705];  # B matrix

# reading speed flow data from excel file as DataFrame
SF_data=CSV.read("seatac_vms1_vms2_treatment_ctl.csv", DataFrame)

I_state=SF_data[1,2:5]; # chosen initial state
Tf=16 # time-horizon (each step is 15 minute slot)
num_in=2 # number of inputs (use arrival, use departure)
num_state=4 # number of states in the model
JM=Model(Gurobi.Optimizer) # defining the JuMP model

# Defining variables of optimization problem
@variable(JM,U[1:num_in,1:Tf], Bin)  # control actions (use arrival, use departure--binary)
@variable(JM,X[1:num_state, 1:Tf+1])  # state variable

#Defining constraints of the optimization problem
@constraint(JM, X .>= 0) # speed or flow cannot be negative
for i=1:Tf
   @constraint(JM, sum(U[:,i]) <= 1) # at most one action every time-step
end
for i=1:num_state
   @constraint(JM, X[i,1] .== I_state[i]) # defining initial state of the system
end
for i=1:Tf
   @constraint(JM, X[:,i+1] .== A*X[:,i] + B*U[:,i]) # state transition model
end

#objective is to maximize the flow of traffic and speed for all times
@objective(JM,Max,sum(sum(X)))

status_SeaTac=optimize!(JM) # solve the optimization problem

X_value=value.(X)
U_value=value.(U)
