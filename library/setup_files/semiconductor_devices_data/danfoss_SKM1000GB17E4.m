

%% SKM1000GB17E4 (MOSFET)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 2.77;                                         % [V]
Vdon_diode = 1.78;                                      % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 450e-3;                                           % [J] @ Tj = 125°C
Eoff = 370e-3;                                          % [J] @ Tj = 125°C
Erec = 200e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 900;                                   % [V]
Ion_sw_losses = 1000;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 34.0/1000;                              % [K/W]
Rth_switch_CH = 16.0/1000;                              % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 43.0/1000;                               % [K/W]
Rth_diode_CH = 17.0/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 800;                                              % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
% ------------------------------------------------------------