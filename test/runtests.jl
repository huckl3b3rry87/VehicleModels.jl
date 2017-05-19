using VehicleModels
using Base.Test

tol=0.0001;

# test the Three DOF Vehicle Model
include(Pkg.dir("VehicleModels/test/ThreeDOF.jl"))
@test_approx_eq_eps(threeDOF_test1(),0,tol)
