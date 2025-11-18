clc


phi_m = 0.2;
psi_ref = 1;
Lq = 0.274e-3;
Ld = 0.263e-3;
iq = Ibez*0.3;
tau_nom = 1140;
Kt = 3.35;
Ibez = tau_nom/Kt*sqrt(2);


id_ref =-abs((phi_m*psi_ref/2/(Lq-Ld)-...
    sqrt((phi_m*psi_ref)^2/4/(Lq-Ld)^2+(iq)^2))/Ibez)