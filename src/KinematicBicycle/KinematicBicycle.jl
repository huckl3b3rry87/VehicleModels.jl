include("parameters.jl")

"""
--------------------------------------------------------------------------------------\n
Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
Modified for NLOptControl.jl by: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/9/2017, Last Modified: 6/30/2017 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering angle and longitudinal acceleration
"""
function KinematicBicycle(pa::VparaKB)

  @unpack_VparaKB pa # vehicle parameters

  dx=Array{Expr}(4);
  # Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series, Spring, 2011, page 2
  dx[1]=:(ux[j]*cos(psi[j] + (atan($la/($la+$lb)*tan(sa[j])))));   # X position
  dx[2]=:(ux[j]*sin(psi[j] + (atan($la/($la+$lb)*tan(sa[j])))));   # Y position
  dx[3]=:((ux[j]/$lb)*sin((atan($la/($la+$lb)*tan(sa[j])))));      # Yaw Angle
  dx[4]=:(ax[j]);                                                  # Longitudinal Speed

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

    f = (dx,x,p,t) -> begin

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
  DifferentialEquations.solve(prob,Tsit5())
end
