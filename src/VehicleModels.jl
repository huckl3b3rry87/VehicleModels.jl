module VehicleModels

using Media, DifferentialEquations

#const threebody_μ = parse(BigFloat,"0.012277471"); const threebody_μ′ = 1 - threebody_μ
 # eventually find a better way to refernce these!
main_dir = "/home/febbo/Documents/workspace/OCP";
func_dir = string(main_dir,"/functions/")# model functions
include(string(func_dir,"F_YF.jl"))   # tire force functions
include(string(func_dir,"F_YR.jl"))
init_dir = string(main_dir,"/simulation/init/")
include(string(init_dir,"Parameters.jl")) # set up parameters
include(string(init_dir,"states.jl"))

export
# Function
  Three_DOF

#############################
# types/functions/constants #
#############################
function Three_DOF(args...)
  if length(args) < 1    # use defaults
    x0 = [200,0,pi/2,0,15,0,0,0]; # for testing
    plot_on = true
    println("testing 3DOF!!\n")
  elseif length(args) == 1
    x0 = args[1:8]
    plot_on = false
  end

    f = (t,x,dx) -> begin
    # STATES:
    # 1. X position
    X        	= x[1];
    # 2. Y position
    Y       	= x[2];
    # 3. Lateral Speed
    V           = x[3];
    # 4. Yaw Rate
    r           = x[4];
    # 5. Steering Angle
    SA          = x[5];
    # 6. Yaw angle
    PSI         = x[6];
    # 7. Longitudinal Speed
    U           = x[7];
    # 8. Longitudinal Acceleration
    Ax          = x[8];

    # Controls
    SR          = 0;  # interp1f(CMD_TM, CMD_SR, t, 'pchip');
    Jx          = 0;  # interp1f(CMD_TM, CMD_Jx, t, 'pchip');

    dx[1]   = U*cos(PSI) - (V + la*r)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*r)*cos(PSI);    # Y position
    dx[3]   = (F_YF(V, U, Ax, r, SA) + F_YR(V, U, Ax, r, SA))/m - r*U;                # Lateral Speed
    dx[4]  	= (la*F_YF(V, U, Ax, r, SA)-lb*F_YR(V, U, Ax, r, SA))/Izz;                # Yaw Rate
    dx[5]   = SR;                                  # Steering Angle
    dx[6]  	= r;                                   # Yaw Angle
    dx[7]  	= Ax;                                  # Longitudinal Speed
    dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  prob = ODEProblem(f, x0)
  sol = solve(prob)
  plot(sol)
end


end # module
