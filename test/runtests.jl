using VehicleModels
using Base.Test
using NLOptControl

tol = 0.5

# test the Three DOF Vehicle Model
include("ThreeDOF.jl")

n = NLOpt(); # constructor for test
n.ocp.params = [Vpara()];
@test threeDOFv1_test1(n) ≈ 0 atol=tol
@test threeDOFv2_test1(n) ≈ 0 atol=tol
@test threeDOFv3_test1(n) ≈ 0 atol=tol
