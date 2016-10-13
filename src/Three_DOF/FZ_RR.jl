@def FZ_RR begin #(N, F_yf, F_yr, V, Ax, r)
	0.5*(FzR0 + KZX*(Ax - V*r)) + KZYR*((@F_YF() + @F_YR())/m)
#=
	ODE_solve = false
	# rear right vertical tire force
	FZ_rr = zeros(Float64, (N+1,1))
	for ii in 1:N+1
		FZ_rr[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[i]*r[ii])) + KZYR*((@F_YF() + @F_YR())/m)
	end
	FZ_rr
	=#
end
