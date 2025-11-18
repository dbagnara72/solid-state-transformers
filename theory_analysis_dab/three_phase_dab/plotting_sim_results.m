close all
clc

tratto1=2;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time_tc_sim(end) - Nc*tc;
t2c = time_tc_sim(end);
t1s = time_ts_dab_sim(end) - Ns_dab*ts_dab;
t2s = time_ts_dab_sim(end);

fontsize_plotting = 12;

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_output_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_tc_sim,dab_voltage_output_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('DAB output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dab}^{out}$','$u_{dab}^{out}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_input_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_tc_sim,dab_voltage_input_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('DAB input current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dab}^{in}$','$u_{dab}^{in}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_ui_output_input','-depsc');
print('dab_ui_output_input','-dpdf');
