using VehicleModels, DataFrames

t0=0.0; tf=20.0;
t=t0:0.01:tf;
U1=zeros(length(t),2);
sa(tt)=.15*sin(3*tt);
U1[:,1]=sa(t)
U1[:,2]=10.0;
X0=[0.0, 0.0, 0.0, 0.0,1.2038];

sol=ThreeDOFv1(Vpara(),Vector(X0),Vector(t),U,t0,tf);
x=sol[:,1];
y=sol[:,2];

U=zeros(length(t),2);
sr(tt)=0.45*cos(3*tt);
U[:,1]=sr(t)
U[:,2]=0.0;
X0=[0.0, 0.0, 0.0, 0.0, 1.2038,0.0,10.0,0.0];
sol=ThreeDOFv2(Vpara(),Vector(X0),Vector(t),U,t0,tf);
x2=sol[:,1];
y2=sol[:,2];

using Plots
gr()
plot(x,y,label="v1")
plot!(x2,y2,label="v2")
