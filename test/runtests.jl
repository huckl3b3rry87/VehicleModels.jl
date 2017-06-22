using VehicleModels
using Base.Test

tol=0.0001;

# test the Three DOF Vehicle Model
include(Pkg.dir("VehicleModels/test/ThreeDOF.jl"))

@test threeDOF_test1() ≈ 0 atol=tol
