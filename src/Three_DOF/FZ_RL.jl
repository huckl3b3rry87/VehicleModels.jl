@def FZ_RL begin #(N, F_yf, F_yr, V, Ax, r)
	0.5*(FzR0 + KZX*(Ax - V*r)) - KZYR*((@F_YF() + @F_YR())/m)
#=
	ODE_solve = false
	# rear left vertical tire force
	FZ_rl = zeros(Float64, (N+1,1))
	for ii in 1:N+1
		FZ_rl[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[ii]*r[ii])) - KZYR*((@F_YF() + @F_YR())/m)
	end
	FZ_r=#
end
