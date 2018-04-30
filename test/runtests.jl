using VehicleModels
using Base.Test
using NLOptControl

tol = 0.5

# test the Three DOF Vehicle Model
include("ThreeDOF.jl")

# constructor for test
n = NLOpt()
n.ocp.params = [Vpara()]
@test threeDOFv1_test1(n) ≈ 0 atol=tol
@test threeDOFv2_test1(n) ≈ 0 atol=tol
