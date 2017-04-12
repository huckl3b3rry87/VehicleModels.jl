using VehicleModels, DataFrames

t0=0.0; tf=20.0;
t=Vector(Ranges.linspace(t0,tf,100));
U1=zeros(length(t),2);
sa(tt)=.15*sin(3*tt);
U1[:,1]=sa(t)
U1[:,2]=10.0;
X0=[0.0, 0.0, 0.0, 0.0,1.2038];

sol=ThreeDOFv1(Vpara(),Vector(X0),t,U1,t0,tf);
x=sol[:,1];
y=sol[:,2];

U=zeros(length(t),2);
sr(tt)=0.45*cos(3*tt);
U[:,1]=sr(t)
U[:,1]=0.0
U[:,2]=0.0;
X0=[0.0, 0.0, 0.0, 0.0, 1.2038,0.0,10.0,0.0];
sol=ThreeDOFv2(Vpara(),Vector(X0),t,U,t0,tf);
x2=sol[:,1];
y2=sol[:,2];

using Plots
gr()
plot(x,y,label="v1")
plot!(x2,y2,label="v2")


t = [0.0,0.0604297,0.196798,0.395295,0.636471,0.896716,1.15055,1.37314,1.54269,1.64265,1.66667,1.76044,1.96707,2.25437,2.57859,2.89035,3.1422,3.29587,3.33333,3.49756,3.84089,4.27004,4.66998,4.93365,5.0];
U=zeros(length(t),2);
SR=copy((0.0-0.0)/0.01)*ones(length(t),);  # first order approximation of SR
JX=0.0;
U[:,1]=SR;U[:,2]=JX;
