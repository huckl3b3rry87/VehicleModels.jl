using VehicleModels
using Base.Test

tol=0.1;

# test the Three DOF Vehicle Model
include("ThreeDOF.jl")

# constructor for test
type N
  params
end
function N()
  N([Vpara()])
end
n=N();

@test threeDOFv1_test1(n) ≈ 0 atol=tol
@test threeDOFv2_test1(n) ≈ 0 atol=tol
