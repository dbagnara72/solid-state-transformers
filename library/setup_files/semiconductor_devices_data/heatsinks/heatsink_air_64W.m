
% HeatSink_1
weigth = 0.150/10;                  % kg - when /10 thermal inertia is not accounted 
cp_al = 880;                        % specific heat_capacity J/K/kg - alluminium
heat_capacity = cp_al*weigth        % J/K
thermal_conducibility_al = 204;     % W/(m K) - alluminium
Rth_switch_HA = 10/8000*6           % K/W
Rth_mosfet_HA = Rth_switch_HA
Tambient = 40;
DThs_init = 0;