
%% FF180R23IE7 (IGBT)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 2.15;                                         % [V]
Vdon_diode = 3;                                         % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 815e-3;                                           % [J] @ Tj = 125°C
Eoff = 1116e-3;                                         % [J] @ Tj = 125°C
Erec = 135e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 1200;                                  % [V]
Ion_sw_losses = 1800;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 17.7/1000;                              % [K/W]
Rth_switch_CH = 10.4/1000;                              % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 39.3/1000;                               % [K/W]
Rth_diode_CH = 15.4/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 1870;                                             % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
gate_charge = 14.6e-6;                                  % [C]
% ------------------------------------------------------------