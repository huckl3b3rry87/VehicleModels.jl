
# steering angle model test
function threeDOFv1_test1(n)
  # generic time span
  t0=0.0; tf=20.0; t=Vector(linspace(t0,tf,100));
  U=zeros(length(t),2);
  sa(tt)=.15*sin(3*tt);
  U[:,1]=sa.(t)
  U[:,2]=10.0;
  X0=[0.0, 0.0, 0.0, 0.0,1.2038];

  sol, U=ThreeDOFv1(n,Vector(X0),t,U,t0,tf);
  actual=sol(sol.t[end]);
  expected=[42.7152,194.619,-0.213113,-0.0127374,1.50199];
  A=zeros(length(actual));
  for n in 1:length(expected)
    A[n]=abs(expected[n]-actual[n]);
  end
  return maximum(A)
end

# steering rate test
function threeDOFv2_test1(n)
  # generic time span
  t0=0.0; tf=20.0; t=Vector(linspace(t0,tf,100));

  U=zeros(length(t),2);
  sr(tt)=0.45*cos(3*tt);
  U[:,1]=sr.(t)
  U[:,1]=0.0;
  U[:,2]=0.0;
  X0=[0.0, 0.0, 0.0, 0.0, 1.2038,0.0,10.0,0.0];
  sol, U=ThreeDOFv2(n,Vector(X0),t,U,t0,tf);

  actual=sol(sol.t[end]);
  expected=[72.2972,186.477,-0.033967,4.7157e-5,1.20477,0.0,10.0,0.0];

  A=zeros(length(actual));
  for n in 1:length(expected)
    A[n]=abs(expected[n]-actual[n]);
  end
  return maximum(A)
end
