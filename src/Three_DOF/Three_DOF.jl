# Three DOF Vehicle Model
include("F_YR.jl")
include("F_YF.jl")
include("parameters.jl")

# other functions to export
include("FZ_RL.jl")
include("FZ_RR.jl")
include("Ax_max.jl")
include("Ax_min.jl")

# this vehicle model is controlled using steering angle
function ThreeDOF{T<:Any}(mdl::JuMP.Model,n,x::Array{T,2},u::Array{T,2},params)
  if n.integrationMethod==:tm; L=size(x)[1]; else; L=size(x)[1]-1; end
  dx = Array(Any,L,n.numStates)
  v = x[:,3]; r = x[:,4]; psi = x[:,5];

  # parameters
  ax = zeros(length(psi)); sa = u[:,1];  # steering angle  TODO pass interval information
  pa=params[1]; ux=params[2]; # for now we assume ux = constant, but -> could do ux[i]!
  @unpack_Vpara pa            # vehicle parameters

  # nonlinear tire model TODO consider using L instead of n.numStatePoints
  @NLexpression(mdl,FYF[i = 1:n.numControlPoints],(PD2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))*sin(PC1*atan((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)) - ((PE2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PE1)*(1 - PE3)*(((atan((v[i] + la*r[i])/(ux+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1))/((((atan((v[i] + la*r[i])/(ux+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)^2 + EP^2)^(0.5)))*((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)) - atan((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)))))) + (PV2*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PV1*(FzF0 - (ax[i] - v[i]*r[i])*dFzx_coeff)));
  @NLexpression(mdl,FYR[i = 1:n.numControlPoints],(PD2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))*sin(PC1*atan((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)) - ((PE2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PE1)*(1 - PE3*(((atan((v[i] - lb*r[i])/(ux+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1))/((((atan((v[i] - lb*r[i])/(ux+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)^2 + EP^2)^(0.5))))*((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)) - atan((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff) + PH1)))))) + (PV2*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)^2 + PV1*(FzR0 + (ax[i] - v[i]*r[i])*dFzx_coeff)));

  # vertical tire load
  @NLconstraint(mdl, FZ_rl_con[i=1:n.numControlPoints], 0 <=  0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) - KZYR*((FYF[i] + FYR[i])/m) - Fz_min)
  @NLconstraint(mdl, FZ_rr_con[i=1:n.numControlPoints], 0 <=  0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) + KZYR*((FYF[i] + FYR[i])/m) - Fz_min)

  # linear tire and for now this also constrains the nonlinear tire model
  # for now constrain linear tire model force
  Caf =  -8.4138e+04;	# cornering stiffness--front axle (N/rad) TODO move these parameters to VehicleModels.jl
  Car =  -7.8126e+04;	# cornering stiffness-- rear axle (N/rad)
  @NLconstraint(mdl, Fyf_con[i=1:n.numControlPoints], -7500 <=  (atan((v[i] + la*r[i])/(ux+EP)) - sa[i])*Caf <= 7500)
  @NLconstraint(mdl, Fyr_con[i=1:n.numControlPoints], -7500 <=   atan((v[i] - lb*r[i])/(ux+EP))*Car <= 7500)

  dx[:,1] = @NLexpression(mdl, [j=1:L], ux*cos(psi[j]) - (v[j] + la*r[j])*sin(psi[j]));    # X position
  dx[:,2] = @NLexpression(mdl, [j=1:L], ux*sin(psi[j]) + (v[j] + la*r[j])*cos(psi[j]));    # Y position
  dx[:,3] = @NLexpression(mdl, [j=1:L], (FYF[j] + FYR[j])/m - r[j]*ux);                    # Lateral Speed
  dx[:,4] = @NLexpression(mdl, [j=1:L], (la*FYF[j]-lb*FYR[j])/Izz);                          # Yaw Rate
  dx[:,5] = @NLexpression(mdl, [j=1:L], r[j]);                                               # Yaw Angle
  return dx
end

function Three_DOF(pa::Vpara,
                   x0::Vector,
                   t::Vector,
                   SA::Vector,
                   t0::Float64,
                   tf::Float64)
error("fix x0! and finish this before using!")
    @unpack_Vpara pa

    # create splines
    sp_SA=Linear_Spline(t,SA);
    sp_U=Linear_Spline(t,U);

    f = (t,x,dx) -> begin
    # states
    X	  = x[1];  # 1. X position
    Y	  = x[2];  # 2. Y position
    V   = x[3];  # 3. Lateral Speed
    R   = x[4];  # 4. Yaw Rate
    #SA  = x[5];  # 5. Steering Angle
    PSI = x[5];  # 6. Yaw angle
    #U   = x[7];  # 7. Longitudinal Speed
    #Ax  = x[6];  # 8. Longitudinal Acceleration

    # controls
    SA  = sp_SA(t);
    U  = sp_U(t);

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*R)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*R)*cos(PSI);    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - R*U;         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz;         # Yaw Rate
    #dx[5]   = SR;                                  # Steering Angle
    dx[5]  	= r;                                   # Yaw Angle
    #dx[6]  	= Ax;                                  # Longitudinal Speed
    #dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  tspan = (t0,tf)
  prob = ODEProblem(f, x0,tspan)
  solve(prob,RK4())
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
    R   = x[4];  # 4. Yaw Rate
    SA  = x[5];  # 5. Steering Angle
    PSI = x[6];  # 6. Yaw angle
    U   = x[7];  # 7. Longitudinal Speed
    Ax  = x[8];  # 8. Longitudinal Acceleration

    # controls
    #SR  = sp_SR(t); # using Interpolations.jl
    #Jx  = sp_Jx(t); # using Interpolations.jl
    SR  = sp_SR[t];
    Jx  = sp_Jx[t];

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*R)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*R)*cos(PSI);    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - R*U;         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz;         # Yaw Rate
    dx[5]   = SR;                                  # Steering Angle
    dx[6]  	= R;                                   # Yaw Angle
    dx[7]  	= Ax;                                  # Longitudinal Speed
    dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  tspan = (t0,tf)
  prob = ODEProblem(f, x0,tspan)
  solve(prob)
end

# this vehicle model is controlled using speed and steering angle TODO finish this!!
function Three_DOF_2(pa::Vpara,
                   x0::Vector,
                   t::Vector,
                   SA::Vector,
                   U::Vector,
                   t0::Float64,
                   tf::Float64)
error("fix x0! and finish this before using!")
    @unpack_Vpara pa

    # create splines
    sp_SA=Linear_Spline(t,SA);
    sp_U=Linear_Spline(t,U);

    f = (t,x,dx) -> begin
    # states
    X	  = x[1];  # 1. X position
    Y	  = x[2];  # 2. Y position
    V   = x[3];  # 3. Lateral Speed
    r   = x[4];  # 4. Yaw Rate
    #SA  = x[5];  # 5. Steering Angle
    PSI = x[5];  # 6. Yaw angle
    #U   = x[7];  # 7. Longitudinal Speed
    #Ax  = x[6];  # 8. Longitudinal Acceleration

    # controls
    SA  = sp_SA(t);
    U  = sp_U(t);

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*r)*sin(PSI);    # X position
    dx[2] 	= U*sin(PSI) + (V + la*r)*cos(PSI);    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - r*U;         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz;         # Yaw Rate
    #dx[5]   = SR;                                  # Steering Angle
    dx[5]  	= r;                                   # Yaw Angle
    #dx[6]  	= Ax;                                  # Longitudinal Speed
    #dx[8]  	= Jx;                                  # Longitudinal Acceleration
  end
  tspan = (t0,tf)
  prob = ODEProblem(f, x0,tspan)
  solve(prob,RK4())
end
