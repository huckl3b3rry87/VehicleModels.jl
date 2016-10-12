module VehicleModels

using Media, DifferentialEquations, Dierckx, Atom, Plots

# Three DOF Vehicle Model
include("Three_DOF/F_YF.jl")   # tire force functions
include("Three_DOF/F_YR.jl")
include("Three_DOF/parameters.jl") # set up parameters
include("Three_DOF/initial_states.jl")

# funcitons in the VehicleModels.jl package
include("utils.jl")

export
  # Objects
  Vehicle_Parameters,
  Tire_Parameters,

# Functions
  Three_DOF,
  Linear_Spline,
  F_YF,
  F_YR,
  FZ_RL,
  FZ_RR,
  Ax_min,
  Ax_max
# MAKE SURE YOU REMOVE THE FINAL COMMA!!

#############################
# types/functions/constants #
#############################
abstract Abstract_Model # Model class
immutable Vehicle_Parameters <: Abstract_Model
  m::Float64
  Izz::Float64
  la::Float64
  lb::Float64
  FzF0::Float64
  FzR0::Float64
  dFzx_coeff::Float64
  KZX::Float64
  KZYR::Float64
  AXC::Array{Float64,1}   # defines polynominal for acceleration bounds
end

function Vehicle_Parameters() # Default constructor
  Vehicle_Parameters(m,
                     Izz,
                     la,
                     lb,
                     FzF0,
                     FzR0,
                     dFzx_coeff,
                     KZX,
                     KZYR,
                     AXC,
                    )
end

immutable Tire_Parameters <: Abstract_Model
  FZ0::Float64
  PCY1::Float64           #Shape factor Cfy for lateral forces
  PDY1::Float64           #Lateral friction Muy
  PDY2::Float64           #Variation of friction Muy with load
  PEY1::Float64           #Lateral curvature Efy at Fznom
  PEY2::Float64           #Variation of curvature Efy with load
  PEY3::Float64           #Zero order camber dependency of curvature Efy
  PKY1::Float64           #Maximum value of stiffness Kfy/Fznom
  PKY2::Float64           #Load at which Kfy reaches maximum value
  PHY1::Float64           #Horizontal shift Shy at Fznom
  PHY2::Float64           #Variation of shift Shy with load
  PVY1::Float64           #Vertical shift in Svy/Fz at Fznom
  PVY2::Float64           #Variation of shift Svy/Fz with load
  PC1::Float64
  PD1::Float64
  PD2::Float64
  PE1::Float64
  PE2::Float64
  PE3::Float64
  PK1::Float64
  PK2::Float64
  PH1::Float64
  PH2::Float64
  PV1::Float64
  PV2::Float64
end

function Tire_Parameters() # Default constructor
  Tire_Parameters(FZ0,
                  PCY1,
                  PDY1,
                  PDY2,
                  PEY1,
                  PEY2,
                  PEY3,
                  PKY1,
                  PKY2,
                  PHY1,
                  PHY2,
                  PVY1,
                  PVY2,
                  PC1,
                  PD1,
                  PD2,
                  PE1,
                  PE2,
                  PE3,
                  PK1,
                  PK2,
                  PH1,
                  PH2,
                  PV1,
                  PV2,
                  )
end

function Three_DOF(vp::VehicleModels.Vehicle_Parameters,
                   tp::VehicleModels.Tire_Parameters,
                   x0::Vector,
                   t::Vector,
                   SR::Vector,
                   Jx::Vector)
    #plot_on = true
    t0=t[1];tf=t[end];

    # create splines
    sp_SR=Linear_Spline(t,SR);
    sp_Jx=Linear_Spline(t,Jx);

    f = (t,x,dx) -> begin
    # STATES:
    X	  = x[1];  # 1. X position
    Y	  = x[2];  # 2. Y position
    V   = x[3];  # 3. Lateral Speed
    r   = x[4];  # 4. Yaw Rate
    SA  = x[5];  # 5. Steering Angle
    PSI = x[6];  # 6. Yaw angle
    U   = x[7];  # 7. Longitudinal Speed
    Ax  = x[8];  # 8. Longitudinal Acceleration

    # Controls
    SR  = sp_SR(t);
    Jx  = sp_Jx(t);

    dx[1]   = U*cos(PSI) - (V + la*r)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*r)*cos(PSI);    # Y position
    dx[3]   = (F_YF(V, U, Ax, r, SA) + F_YR(V, U, Ax, r, SA))/m - r*U;                # Lateral Speed
    dx[4]  	= (la*F_YF(V, U, Ax, r, SA)-lb*F_YR(V, U, Ax, r, SA))/Izz;                # Yaw Rate
    dx[5]   = SR;                                  # Steering Angle
    dx[6]  	= r;                                   # Yaw Angle
    dx[7]  	= Ax;                                  # Longitudinal Speed
    dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  tspan = [t0,tf]
  prob = ODEProblem(f, x0)
  solve(prob::ODEProblem,tspan)

  #=
  if plot_on
    plot(sol)
    gui()
  end
  =#
end

end # module
