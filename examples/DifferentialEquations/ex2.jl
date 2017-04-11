using VehicleModels, DataFrames,

t0=0; tf=10;

t=readtable("t.csv");
u=readtable("U.csv");
X0=readtable("X0.csv");

sol=ThreeDOFv2(Vpara(),Vector(X0[:X0]),Vector(t[:t]),Vector(u[:U1]),Vector(u[:U2]),t0,tf);
function savePlantData(r)
  dfs=DataFrame();
  temp = [r.dfs_plant[jj][:t][1:end-1,:] for jj in 1:length(r.dfs_plant)]; # time
  U=[idx for tempM in temp for idx=tempM]; dfs[:t]=U;

  for st in 1:n.numStates # state
    temp = [r.dfs_plant[jj][n.state.name[st]][1:end-1,:] for jj in 1:length(r.dfs_plant)];
    U=[idx for tempM in temp for idx=tempM];
    dfs[n.state.name[st]]=U;
  end

  for ctr in 1:n.numControls # control
    temp = [r.dfs_plant[jj][n.control.name[ctr]][1:end-1,:] for jj in 1:length(r.dfs_plant)];
    U=[idx for tempM in temp for idx=tempM];
    dfs[n.control.name[ctr]]=U;
  end
  cd(r.results_dir)
    writetable("plant_data.csv",dfs);
  cd(r.main_dir)
  return nothing
end
