using VehicleModels, Plots, DataFrames
using PrettyPlots
# Name For Resutls
results_dir = string("testing2");

dfs = Vector{DataFrame}(2) # create am empty DataFrame

# read in sample data
s_data = readtable("settings_data.csv")
obs_data = readtable("obstacle_data.csv")
dfs[1] = readtable("STATES.csv") # according to optimization
t_data = float(dfs[1][:t]);
t_data[1:41] = 0:0.1:4;
t_data=t_data[1:41];
SR_data = float(dfs[1][:SR]);
Jx_data = float(dfs[1][:Jx]);
t_data=Vector{Float64}=0:0.1:40;
X0=[0.0, 0.0, 0.0, 0.0, 1.2037,0.0,10.0,0.0];
pa = Vpara(); # initialize parameter set
t0=0.0;
tf=0.01;
sol = ThreeDOFv2(pa,x0,t_data[1:41],SR_data[1:41],Jx_data[1:41],t0,tf)
sol(tf)[:]
