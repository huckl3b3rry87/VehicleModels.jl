# define model parameters
m          = 2.6887e+03
Izz        = 4.1101e+03
la         = 1.5775        # distance from CoG to front axle
lb         = 1.7245        # distance from CoG to rear axle
FzF0       = 1.3680e+04
FzR0       = 1.2696e+04
dFzx_coeff = 705
KZX        = 705
KZYR       = 1040

# tire data
FZ0     = 35000.0;
PCY1    = 1.5874;               #Shape factor Cfy for lateral forces
PDY1    = 0.73957;              #Lateral friction Muy
PDY2    = -0.075004;            #Variation of friction Muy with load
PEY1    = 0.37562;              #Lateral curvature Efy at Fznom
PEY2    = -0.069325;            #Variation of curvature Efy with load
PEY3    = 0.29168;              #Zero order camber dependency of curvature Efy
PKY1    = -10.289;              #Maximum value of stiffness Kfy/Fznom
PKY2    = 3.3343;               #Load at which Kfy reaches maximum value
PHY1    = 0.0056509;            #Horizontal shift Shy at Fznom
PHY2    = -0.0020257;           #Variation of shift Shy with load
PVY1    = 0.015216;             #Vertical shift in Svy/Fz at Fznom
PVY2    = -0.010365;            #Variation of shift Svy/Fz with load

PC1     = PCY1;
PD1     = PDY1 - PDY2;
PD2     = PDY2/FZ0;
PE1     = PEY1 - PEY2;
PE2     = PEY2/FZ0;
PE3     = PEY3;
PK1     = PKY1*FZ0;
PK2     = 1/(PKY2*FZ0);
PH1     = PHY1 - PHY2;
PH2     = PHY2/FZ0;
PV1     = PVY1 - PVY2;
PV2     = PVY2/FZ0;

# defines polynominal for acceleration bounds
AXC =[-0.000128015180401862,	0.00858618595422724,	-0.225657108071454,	3.08283259993589,	-0.000138537090018958,	0.00684702729623608,	-0.120391102052425,	-3.55886697370079]

##TO DO!! Finidh parameter set
# weights
w_goal = 1; # should be zero when vehicle is not within goal distance during prediction horizon
w_psi = 0.01;
w_time = 0.05;
w_haf = 1.0e-5;
w_Fz = 0.1;  #0.5 in paper
w_ce = 1;
w_sa = 0.1;
w_sr = 1;
w_jx = 0.01;

sm = 5 # m add distance to make sure we don't hit obstacle
Fz_min          = 1000;
Fz_off          = 100;
a_t = Fz_min + 3*Fz_off;  # soft tire force constraint constants
b_t = Fz_off;
EP = 0.001
