module VehicleModels

using Media, DifferentialEquations, Dierckx, Atom, Plots, Parameters

macro def(name, definition)
  return quote
    macro $name()
      esc($(Expr(:quote,definition)))
    end
  end
end

# Three DOF Vehicle Model
include("Three_DOF/F_YR.jl")
include("Three_DOF/F_YF.jl")

# other functions to export
include("Three_DOF/FZ_RL.jl")
include("Three_DOF/FZ_RR.jl")
include("Three_DOF/Ax_max.jl")
include("Three_DOF/Ax_min.jl")

# funcitons in the VehicleModels.jl package
include("utils.jl")

export
  # Objects
  Vpara,

  # Functions
  Three_DOF,
  Linear_Spline,

  # Macros and support functions
  @F_YF,
  @F_YR,
  @FZ_RL,
  @FZ_RR,
  @Ax_min,
  @Ax_max,
  @unpack_Vpara,
  @pack_Vpara
  # MAKE SURE YOU REMOVE THE FINAL COMMA!!

#############################
# types/functions/constants #
#############################
@with_kw immutable Vpara @deftype Float64
    # define model parameters
    m          = 2.6887e+03
    Izz        = 4.1101e+03
    la         = 1.5775        # distance from CoG to front axle
    lb         = 1.7245        # distance from CoG to rear axle
    FzF0       = 1.3680e+04
    FzR0       = 1.2696e+04
    dFzx_coeff = 705.
    KZX        = 705.
    KZYR       = 1040.

    # defines polynominal for acceleration bounds
    AXC::Array{Float64,1} = [-0.000128015180401862,	0.00858618595422724,	-0.225657108071454,	3.08283259993589,	-0.000138537090018958,	0.00684702729623608,	-0.120391102052425,	-3.55886697370079]

    # vehicle Limits
    x_min = 0.
    x_max = 400.
    y_min = 0.
    y_max = 400.
    sa_min = -30*pi/180
    sa_max = 30*pi/180
    psi_min = -2*pi
    psi_max = 2*pi
    u_min = 5.
    u_max = 29.
    sr_min = -5*pi/180
    sr_max = 5*pi/180
    jx_min = -5.
    jx_max = 5.

    # tire parameters
    FZ0     = 35000.0;
    PCY1    = 1.5874;               #Shape factor Cfy for lateral forces
    PDY1    = 0.73957;              #Lateral friction Muy
    PDY2    = -0.075004;            #Variation of friction Muy with load
    PEY1    = 0.37562;              #Lateral curvature Efy at Fznom
    PEY2    = -0.069325;            #Variation of curvature Efy with load
    PEY3    = 0.29168;              #Zero order camber dependency of curvature Efy
    PKY1    = -10.289;              #Maximum value of stiffness Kfy/Fznom
    PKY2    = 3.3343;               #Load at which Kfy reaches maximum value
    PHY1    = 0.0056509;            #Horizontal shift Shy at Fznom
    PHY2    = -0.0020257;           #Variation of shift Shy with load
    PVY1    = 0.015216;             #Vertical shift in Svy/Fz at Fznom
    PVY2    = -0.010365;            #Variation of shift Svy/Fz with load
    PC1     = PCY1;
    PD1     = PDY1 - PDY2;
    PD2     = PDY2/FZ0;
    PE1     = PEY1 - PEY2;
    PE2     = PEY2/FZ0;
    PE3     = PEY3;
    PK1     = PKY1*FZ0;
    PK2     = 1/(PKY2*FZ0);
    PH1     = PHY1 - PHY2;
    PH2     = PHY2/FZ0;
    PV1     = PVY1 - PVY2;
    PV2     = PVY2/FZ0;

    # constrained initial states
    x0_     = 200.;
    y0_     = 0.;
    psi0_   = pi/2;
    v0_     = 0.;
    u0_     = 15.;
    sa0_    = 0.; # adding additional initial states
    sr0_    = 0.;
    ax0_    = 0.;
    jx0_    = 0.;
    r0_     = 0.;

    ##TODO: Probably want to move some of these parameters somewhere else?

    # weights
    w_goal = 1.; # should be zero when vehicle is not within goal distance during prediction horizon
    w_psi = 0.01;
    w_time = 0.05;
    w_haf = 1.0e-5;
    w_Fz = 0.1;  #0.5 in paper
    w_ce = 1.;
    w_sa = 0.1;
    w_sr = 1.;
    w_jx = 0.01;

    sm = 5. # m add distance to make sure we don't hit obstacle
    Fz_min          = 1000.;
    Fz_off          = 100.;
    a_t = Fz_min + 3*Fz_off;  # soft tire force constraint constants
    b_t = Fz_off;
    EP = 0.001
end

function Three_DOF(pa::Vpara,
                   x0::Vector,
                   t::Vector,
                   SR::Vector,
                   Jx::Vector,
                   t0::Float64,
                   tf::Float64)
    @unpack_Vpara pa

    # create splines
    sp_SR=Linear_Spline(t,SR);
    sp_Jx=Linear_Spline(t,Jx);

    f = (t,x,dx) -> begin
    # states
    X	  = x[1];  # 1. X position
    Y	  = x[2];  # 2. Y position
    V   = x[3];  # 3. Lateral Speed
    r   = x[4];  # 4. Yaw Rate
    SA  = x[5];  # 5. Steering Angle
    PSI = x[6];  # 6. Yaw angle
    U   = x[7];  # 7. Longitudinal Speed
    Ax  = x[8];  # 8. Longitudinal Acceleration

    # controls
    SR  = sp_SR(t);
    Jx  = sp_Jx(t);

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*r)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*r)*cos(PSI);    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - r*U;         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz;         # Yaw Rate
    dx[5]   = SR;                                  # Steering Angle
    dx[6]  	= r;                                   # Yaw Angle
    dx[7]  	= Ax;                                  # Longitudinal Speed
    dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  tspan = [t0,tf]
  prob = ODEProblem(f, x0)
  solve(prob::ODEProblem,tspan,alg=:RK4)
end
end # module
