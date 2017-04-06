using VehicleModels, DataFrames

t0=3.6; tf=3.8;

t=readtable("t.csv");
u=readtable("U.csv");
X0=readtable("X0.csv");

sol=ThreeDOFv2(Vpara(),Vector(X0[:X0]),Vector(t[:t]),Vector(u[:U1]),Vector(u[:U2]),t0,tf);

X0p=sol(tf)[:];
