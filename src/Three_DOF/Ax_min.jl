@def Ax_min begin #(U)

	# minimum longitudinal acceleration for given speed
	Ax_min = Array(Float64,(length(U),1))
	for i in eachindex(U)
		Ax_min[i,1] = AXC[5]*U[i]^3 + AXC[6]*U[i]^2 + AXC[7]*U[i] + AXC[8]
	end
	Ax_min
end
