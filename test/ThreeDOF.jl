
# steering angle model test
function threeDOFv1_test1(n)
  # generic time span
  t0 = 0.0; tf = 20.0; t = Vector(linspace(t0,tf,100));
  U = zeros(length(t),2)
  sa(tt) = 0.15*sin(3*tt)
  U[:,1] = sa.(t)
  U[:,2] = 10.0
  X0 = [0.0, 0.0, 0.0, 0.0, 1.2038]
  sol, U = ThreeDOFv1(n,Vector(X0),t,U,t0,tf)
  actual = sol(sol.t[end])
  expected = [40.8714, 195.019, -0.170866, -0.0139167, 1.5131]
  A = zeros(length(actual))
  for i in 1:length(expected)
    A[i] = abs(expected[i]-actual[i])
  end
  return maximum(A)
end

# steering rate test
function threeDOFv2_test1(n)
  # generic time span
  t0 = 0.0; tf = 20.0; t = Vector(linspace(t0,tf,100));
  U = zeros(length(t),2)
  sr(tt) = 0.45*cos(3*tt)
  U[:,1] = sr.(t)
  U[:,1] = 0.0
  U[:,2] = 0.0
  X0 = [0.0, 0.0, 0.0, 0.0, 1.2038, 0.0, 10.0, 0.0]
  sol, U = ThreeDOFv2(n,Vector(X0),t,U,t0,tf)
  actual = sol(sol.t[end])
  expected = [71.7627,186.682, 0.0, 0.0, 1.2038, 0.0, 10.0, 0.0]
  A = zeros(length(actual))
  for i in 1:length(expected)
    A[i] = abs(expected[i]-actual[i])
  end
  return maximum(A)
end
