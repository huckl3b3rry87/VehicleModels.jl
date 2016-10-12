function Ax_min(U)

	# minimum longitudinal acceleration for given speed
	Ax_min = zeros(Float64, (N+1,1))
	for i in 1:N+1
		Ax_min[i] = AXC[5]*U[i]^3 + AXC[6]*U[i]^2 + AXC[7]*U[i] + AXC[8]
	end
	Ax_min
end
