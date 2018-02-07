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
  for i in 1:length(V) #TODO put an error message here if V and t are different sizes
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

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2017, Last Modified: 2/06/2018 \n
--------------------------------------------------------------------------------------\n
"""
function checkCrash(n, c, sm; kwargs...)
    kw = Dict(kwargs);

    if !haskey(kw,:plant); plant=false;
    else; plant = get(kw,:plant,0);
    end

    if plant  # NOTE currently not doing any interpolation here to get a more accurate prediction
        t = n.r.dfs_plant[end][:t]
        X = n.r.dfs_plant[end][:x]
        Y = n.r.dfs_plant[end][:y]
        crash_tmp = zeros(length(c.o.B),1);
        for obs in length(c.o.B)
            # obstacle postions after the initial postion
            X_obs= c.o.X0[obs] .+ c.o.s_x[obs].*t
            Y_obs= c.o.Y0[obs] .+ c.o.s_y[obs].*t
            if minimum((X-X_obs).^2./(c.o.B[obs]+sm).^2 + (Y-Y_obs).^2./(c.o.A[obs]+sm).^2) < 1
                crash_tmp[obs] = 1
                println(minimum((X-X_obs).^2./(c.o.B[obs]+sm).^2 + (Y-Y_obs).^2./(c.o.A[obs]+sm).^2))
            end
        end
        if maximum(crash_tmp)>0
            crash = true;
            print("the vehicle crashed! \n")
        else
            crash = false;
        end
       return crash
    else # in the NLOptControl.jl paper, there was no plant and the following code was used
        # also this will break, need to remove pts from function handle
       pts = 300;
       # redo interpolation with desired number of points
      if n.s.integrationMethod == :ps
          interpolateLagrange!(n;numPts=Int64(pts/n.Ni))
      else
          interpolateLinear!(n;numPts=pts)
      end
      X = n.r.X_pts[:,1]; Y = n.r.X_pts[:,2];

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
end
