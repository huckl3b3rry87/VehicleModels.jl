function FZ_RL(F_yf, F_yr, V, Ax, Yaw)

	# rear left vertical tire force
	FZ_RL = zeros(Float64, (N+1,1))
	for i in 1:N+1
		FZ_RL[i] = 0.5*(FzR0 + KZX*(Ax[i] - V[i]*Yaw[i])) - KZYR*((F_yf[i] + F_yr[i])/m)
	end
	FZ_RL
end
