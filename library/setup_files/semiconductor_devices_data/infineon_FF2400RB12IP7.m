
%% FF2400RB12IP7 (IGBT - 3-Level)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 1.37;                                         % [V]
Vdon_diode = 1.65;                                         % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 205e-3;                                           % [J] @ Tj = 125°C
Eoff = 845e-3;                                         % [J] @ Tj = 125°C
Erec = 150e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 600;                                  % [V]
Ion_sw_losses = 2400;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 17.9/1000;                              % [K/W]
Rth_switch_CH = 12.9/1000;                              % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 34.1/1000;                               % [K/W]
Rth_diode_CH = 14.7/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 885;                                             % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
gate_charge = 38.1e-6;                                  % [C]
% ------------------------------------------------------------