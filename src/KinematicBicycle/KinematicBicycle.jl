include("parameters.jl")

"""
dx=KinematicBicycle(n,x,u)
--------------------------------------------------------------------------------------\n
Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
Modified for NLOptControl.jl by: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/9/2017, Last Modified: 5/31/2017 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering angle and longitudinal acceleration
"""
function KinematicBicycle{T<:Any}(n,x::Array{T,2},u::Array{T,2})
  if n.s.integrationMethod==:tm; L=size(x)[1]; else; L=size(x)[1]-1; end
  dx = Array(Any,L,n.numStates)
  psi = x[:,3]; ux = x[:,4]; sa = u[:,1]; ax = u[:,2];

  @unpack_VparaKB n.params[1] # vehicle parameters

  # Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series, Spring, 2011, page 2
  dx[:,1] = @NLexpression(n.mdl, [i=1:L], ux[i]*cos(psi[i] + (atan(la/(la+lb)*tan(sa[i])))));   # X position
  dx[:,2] = @NLexpression(n.mdl, [i=1:L], ux[i]*sin(psi[i] + (atan(la/(la+lb)*tan(sa[i])))));   # Y position
  dx[:,3] = @NLexpression(n.mdl, [i=1:L], (ux[i]/lb)*sin((atan(la/(la+lb)*tan(sa[i])))));       # Yaw Angle
  dx[:,4] = @NLexpression(n.mdl, [i=1:L], ax[i]);                                               # Longitudinal Speed
  return dx
end

function KinematicBicycle(pa::VparaKB,
                          x0::Vector,
                           t::Vector,
                           U::Matrix,
                          t0::Float64,
                          tf::Float64)
    @unpack_VparaKB pa # vehicle parameters

    # create splines
    sp_SA=Linear_Spline(t,U[:,1]);
    sp_AX=Linear_Spline(t,U[:,2]);

    f = (t,x,dx) -> begin

    # states
    psi = x[3];  # 3. Yaw Angle
    ux  = x[4];  # 4. Longitudinal Speed

    # controls
    sa  = sp_SA[t]; # Steering Angle
    ax  = sp_AX[t]; # Longitudinal Acceleration

    # diff eqs.
    dx[1] = ux*cos(psi + (atan(la/(la+lb)*tan(sa))));   # 1. X position
    dx[2] = ux*sin(psi + (atan(la/(la+lb)*tan(sa))));   # 2. Y position
    dx[3] = (ux/lb)*sin((atan(la/(la+lb)*tan(sa))));    # 3. Yaw Angle
    dx[4] = ax;                                         # 4. Longitudinal Speed
  end
  tspan = (t0,tf)
  prob = ODEProblem(f,x0,tspan)
  DifferentialEquations.solve(prob,RK4())
end
