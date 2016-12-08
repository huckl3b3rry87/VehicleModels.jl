using VehicleModels, Plots, DataFrames
using PrettyPlots
# Name For Resutls
results_dir = string("testing");

dfs = Vector{DataFrame}(2) # create am empty DataFrame

# read in sample data
s_data = readtable("settings_data.csv")
obs_data = readtable("obstacle_data.csv")
dfs[1] = readtable("STATES.csv") # according to optimization
t_data = float(dfs[1][:t]);
SR_data = float(dfs[1][:SR]);
Jx_data = float(dfs[1][:Jx]);
t0=dfs[1][:t][1];tf=dfs[1][:t][end];
x0 = [dfs[1][:X][1],                 # 1. X, position
      dfs[1][:Y][1],                 # 2. Y, position
      dfs[1][:V][1],                 # 3. V, Lateral Speed
      dfs[1][:r][1],                 # 4. r, Yaw Rate
      dfs[1][:SA][1],                # 5. SA, Steering Angle
      dfs[1][:PSI][1],               # 6. PSI, Yaw angle
      dfs[1][:U][1],                 # 7. U, Longitudinal Speed
      dfs[1][:Ax][1]];               # 8. Ax, Longitudinal Acceleration

pa = Vpara(); # initialize parameter set
sol = Three_DOF(pa,x0,t_data,SR_data,Jx_data,t0,tf)

# extract data
t0=sol.t[1]; tf=sol.t[end]; pts = length(dfs[1][:V]);  #does not have to be same size, just doing this so the animation looks OK
x = [sol(t)[1] for t in linspace(t0,tf,pts)];
y = [sol(t)[2] for t in linspace(t0,tf,pts)];
v = [sol(t)[3] for t in linspace(t0,tf,pts)];
r = [sol(t)[4] for t in linspace(t0,tf,pts)];
sa = [sol(t)[5] for t in linspace(t0,tf,pts)];
psi = [sol(t)[6] for t in linspace(t0,tf,pts)];
u = [sol(t)[7] for t in linspace(t0,tf,pts)];
ax = [sol(t)[8] for t in linspace(t0,tf,pts)];

# build data for plots
t = convert(Vector, linspace(t0,tf,pts));
sp_SR = Linear_Spline(t_data, SR_data);
sp_Jx = Linear_Spline(t_data, Jx_data);
#sr  = sp_SR(t); # using Interpolations.jl
#jx  = sp_Jx(t); # using Interpolations.jl
sr  = sp_SR[t];
jx  = sp_Jx[t];

# put the data into a DataFrame
dfs[2]=x_to_DataFrame(t,x,y,sa,ax,psi,u,v,r,jx,sr) # according to ODE equations (RK4)

# plot the results
@unpack_Vpara pa  # unpack the parameters for the plots

all_plots(dfs,["opt.","RK4"],results_dir,obs_data,s_data,pa)

# simulate
panim_fun(dfs,["opt.","RK4"], results_dir, obs_data,s_data,pa)
