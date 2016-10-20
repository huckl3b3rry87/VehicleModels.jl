function Linear_Spline(t::Vector,V::Vector)
  # t must be the time vector
  # V is any vector that you are interpolating

  # remove any repeating values
  M = Array(Bool,length(t)); M[1:length(t)] = false;
  for i in 1:length(t)
      for q in 1:length(t)
          if t[i]==t[q] && i!=q || M[i]
              M[i]=true
          else
              M[i]=false
          end
      end
  end
  rm_idx = find(M)
  
  if (length(t)==length(rm_idx))
    error("No time has elapsed and there will be an issue with interpolation! -----> Do not even try to simulate this data!")
  end

  # initialize vetors
  t_new = Array(Float64,(length(t)-length(rm_idx)));
  V_new = Array(Float64,(length(t)-length(rm_idx)));

  q=1;
  for i in 1:length(t)
      if !M[i]
          t_new[q] = t[i];
          V_new[q] = V[i];
          q=q+1
      end
  end
  Spline1D(t_new,V_new,k=1)    # linear spline
end
