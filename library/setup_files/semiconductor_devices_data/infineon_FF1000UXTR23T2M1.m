
%% FF1000UXTR23T2M1 (MOSFET)
Vth = 4.5;                                              % [V]
Rds_on = 1.7e-3;                                       % [Ohm]
Vdon_diode = 4.4;                                         % [V]
Vgamma = Vdon_diode;                                             % [V]
Rdon_diode = 0.85e-3;                                   % [Ohm]
Eon = 540e-3;                                            % [J] @ Tj = 125°C
Eoff = 370e-3;                                          % [J] @ Tj = 125°C
Eerr = 40e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 1500;                                  % [V]
Ion_sw_losses = 2000;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_mosfet_JC = 20/1000;                                % [K/W]
Rth_mosfet_CH = 6/1000;                                 % [K/W]
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH;   % [K/W]
Lstray_module = 10e-9;                                  % [H]
Irr = 1100;                                              % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
% ------------------------------------------------------------