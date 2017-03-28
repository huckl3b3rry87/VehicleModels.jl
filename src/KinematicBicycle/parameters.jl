@with_kw immutable VparaKB @deftype Float64
    # define model parameters
    la     = 1.5775        # distance from CoG to front axle
    lb     = 1.7245        # distance from CoG to rear axle

    # vehicle Limits
    x_min    = 0.
    x_max    = 400.
    y_min    = 0.
    y_max    = 400.
    psi_min  = -2*pi
    psi_max  = 2*pi
    u_min    = 5.
    u_max    = 29.
    ax_min   = -2.
    ax_max   = 2.

    sa_min   = -30*pi/180
    sa_max   = 30*pi/180

    # constrained initial states
    x0_     = 200.;
    y0_     = 0.;
    psi0_   = pi/2;
    u0_     = 15.;
    ax0_    = 0.;
end
