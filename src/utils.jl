function Linear_Spline(t::Vector,V::Vector)
  # t must be the time vector
  # V is any vector that you are interpolating

  # remove any repeating values
  M = Array{Bool}(length(t)); M[1:length(t)] = false;
  for i in 1:length(t)-1
      if t[i]==t[i+1]
          M[i]=true
      else
          M[i]=false
      end
  end

  rm_idx = find(M)

  if (length(t)==length(rm_idx))
    error("No time has elapsed and there will be an issue with interpolation! -----> Do not even try to simulate this data!")
  end

  # initialize vetors
  t_new = Array{Float64}(length(t)-length(rm_idx));
  V_new = Array{Float64}(length(t)-length(rm_idx));

  q=1;
  for i in 1:length(V) #TODO could put an error message here if V and t are different sizes
      if !M[i]
          t_new[q] = t[i];
          V_new[q] = V[i];
          q=q+1
      end
  end

  # make interpolant using Dierckx.jl
  #Spline1D(t_new,V_new,k=1)    # linear spline

  # make interpolant using Interpolations.jl
  knots = (t_new,)
  interpolate(knots,V_new,Gridded(Linear()))
end


function checkCrash(n, c, pts, sm)

  interpolateLagrange!(n;numPts=Int64(pts/n.Ni))

  # put X and Y polynomials points into a vector
  temp = [n.r.X_polyPts[1][int] for int in 1:length(n.Nck)];
  X=[idx for tempM in temp for idx=tempM];

  temp = [n.r.X_polyPts[2][int] for int in 1:length(n.Nck)];
  Y=[idx for tempM in temp for idx=tempM];

  crash_tmp = zeros(pts,1);
  for i = 1:pts
    temp = (X[i]-c.o.X0[1])^2/(c.o.B[1]+sm)^2 + (Y[i]-c.o.Y0[1])^2/(c.o.A[1]+sm)^2;
   if temp < 1
      crash_tmp[i] = 1;
   else
       crash_tmp[i] = 0;
   end
  end

  if maximum(crash_tmp)>0
      crash = 1;
      print("the vehicle crashed! \n")
  else
      crash = 0;
  end

 return crash
end
