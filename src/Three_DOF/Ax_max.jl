@def Ax_max begin #(U)

	# maximum longitudinal acceleration for given speed
	Ax_max = Array(Float64,(N+1,1))
	for i in 1:N+1
		Ax_max[i,1] = AXC[1]*U[i]^3 + AXC[2]*U[i]^2 + AXC[3]*U[i] + AXC[4]
	end
	Ax_max
end
