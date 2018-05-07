
"""
--------------------------------------------------------------------------------------\n
Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
Modified for use wiNLOptControl.jl by: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/9/2017, Last Modified: 4/08/2018 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering angle and longitudinal acceleration
"""
function KinematicBicycle_expr(n)

  @unpack_Vpara n.ocp.params[1]

  dx=Array{Expr}(4);
  # Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series, Spring, 2011, page 2
  dx[1]=:(u[j]*cos(psi[j] + (atan($la/($la+$lb)*tan(sa[j])))))   # X position
  dx[2]=:(u[j]*sin(psi[j] + (atan($la/($la+$lb)*tan(sa[j])))))   # Y position
  dx[3]=:(u[j]*cos(atan($la/($la+$lb)*tan(sa[j])))/($la+$lb)*tan(sa[j])   )      # Yaw Angle
  dx[4]=:(a[j])                                                  # Total Speed

  return dx
end

function KinematicBicycle(n,
                          x0::Vector,
                           t::Vector,
                           U::Matrix,
                          t0::Float64,
                          tf::Float64)
    @unpack_Vpara n.ocp.params[1]

    # create splines
    sp_SA = linearSpline(t,U[:,1])
    sp_A = linearSpline(t,U[:,2])

    f = (dx,x,p,t) -> begin

    # states
    psi = x[3]  # 3. Yaw Angle
    u = x[4]   # 4. Total Speed

    # controls
    sa = sp_SA[t] # Steering Angle
    a = sp_A[t] # Total Acceleration

    # diff eqs.
    dx[1] = u*cos(psi + (atan(la/(la+lb)*tan(sa))))   # 1. X position
    dx[2] = u*sin(psi + (atan(la/(la+lb)*tan(sa))))   # 2. Y position
    dx[3] = u*cos(atan(la/(la+lb)*tan(sa)))/(la+lb)*tan(sa)    # 3. Yaw Angle
    dx[4] = a                                         # 4. Total Speed
  end
  tspan = (t0,tf)
  prob = ODEProblem(f,x0,tspan)
  sol = DiffEqBase.solve(prob,Tsit5())
  U = [sp_SA,sp_A]
  return sol, U
end
