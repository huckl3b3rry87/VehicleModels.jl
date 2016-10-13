@def FZ_RL begin #(N, F_yf, F_yr, V, Ax, r)

	# define the lateral tire forces
	F_yf=@F_YF()
	F_yr=@F_YR()
	FZ_rl = zeros(Float64, (length(Ax),1))
	for ii in eachindex(Ax)
		FZ_rl[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[ii]*r[ii])) - KZYR*((F_yf[ii] + F_yr[ii])/m)
	end
	FZ_rl
end


#=
	ODE_solve = false
	# rear left vertical tire force
	FZ_rl = zeros(Float64, (N+1,1))
	for ii in 1:N+1
		FZ_rl[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[ii]*r[ii])) - KZYR*((@F_YF() + @F_YR())/m)
	end
	FZ_r=#
