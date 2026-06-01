clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;                        % Plocha dna nádrží [m^2]
S = 25 * 1e-4;                % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;              % Vstupní průtok [m^3/s]
T_sim = 10000;
Qin_step = 50;
A_inv = 1 / A;
k_valve = S * sqrt(2 * g);
f_sin = 2*pi/(T_sim / 8);     % frekvence kmitání sinosového signálu
Tv = 0.1;                     % časová konstanta pro dynamiku ventilů
h_sp = 0.8155;

% Parametry pro repeating sequence blok
t_vals_h1 = [0 5000 10000];
t_vals_h2 = [0 1000 6000 10000];
out_vals_h1 = [0 1 0];
out_vals_h2 = [0 0 1 0.2];

% Parametry regulačních ventilů (Úkol 3)
z1_0 = 0.5;
z2_0 = 0.5;

% Výpočet ustálených stavů (zahrnující ventily)
h1_calced = (Qin / (z1_0 * k_valve))^2;
h2_calced = (Qin / (z2_0 * k_valve))^2;

% zlomky přenosové funkce ventilů
num = 1;
den = [Tv 1];

% Vnější smyčky (Level Control)
Kp_LC1 = -10; Ki_LC1 = -1;
Kp_LC2 = Kp_LC1; Ki_LC2 = Ki_LC1;
% Vnitřní smyčky (Flow Control)
Kp_FC1 = 10;   Ki_FC1 = 1;
Kp_FC2 = Kp_FC1;   Ki_FC2 = Ki_FC1;

%% MPC regulátor
% Z pohledu nadřazeného MPC se FC chovají téměř ideálně (Q_out = Q_SP).
% Matematický model pro MPC je tedy čistá hmotnostní bilance (integrátory).
% Využijeme state-space model

A_mpc = [0 0; 0 0];
B_mpc = [-A_inv 0; A_inv -A_inv]; % Akční veličiny (MV): řízení průtoků
B_v   = [A_inv; 0];               % Měřená porucha (MD): vstupní průtok
C_mpc = [1 0; 0 1];               % Měřené výstupy (MO): hladiny
D_mpc = zeros(2,2);
D_v   = zeros(2,1);

% Vytvoření stavového modelu a specifikace signálů
sys_mpc = ss(A_mpc, [B_mpc B_v], C_mpc, [D_mpc D_v]);
sys_mpc = setmpcsignals(sys_mpc, 'MV', [1 2], 'MD', 3);
sys_mpc.InputName = {'Qout1_SP', 'Qout2_SP', 'Qin'};
sys_mpc.OutputName = {'h1', 'h2'};

% Vytvoření MPC objektu
Ts_mpc = 1;
mpc_obj = mpc(sys_mpc, Ts_mpc); % Model si interně diskretizuje pokud zadám vzorkovací periodu

% Nastavení horizontů (jak daleko do budoucnosti MPC "vidí" a plánuje)
mpc_obj.PredictionHorizon = 50; % vídí 50 sekund před regulací
mpc_obj.ControlHorizon = 10; % planuje za 10 sekund do

% Fyzikální omezení akčních zásahů (průtok nemůže být záporný, ani větší než max propustnost ventilu)
mpc_obj.MV(1).Min = 0; mpc_obj.MV(1).Max = 0.01;
mpc_obj.MV(2).Min = 0; mpc_obj.MV(2).Max = 0.01;

% Nastavení poč. pracovních bodů
mpc_obj.Model.Nominal.U = [Qin Qin Qin]; % [Q1out, Q2out, Qin_MD]
mpc_obj.Model.Nominal.Y = [h_sp h_sp];     % [h1, h2]


%% Simulink

% Spuštění modelu Simulink
model_name = 'model';

% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

%% Extrakce dat (12 signálů z MUXu)
t = out.h_data.Time;

% Signály (Žádané hodnoty - setpointy)
h1_sp = out.h_data.Data(:, 1);
h2_sp = out.h_data.Data(:, 2);

% PID data
z1_pid = out.h_data.Data(:, 3);
z2_pid = out.h_data.Data(:, 4);
h1_pid = out.h_data.Data(:, 5);
h2_pid = out.h_data.Data(:, 6);
Qin_pid = out.h_data.Data(:, 7);

% MPC data
z1_mpc = out.h_data.Data(:, 8);
z2_mpc = out.h_data.Data(:, 9);
h1_mpc = out.h_data.Data(:, 10);
h2_mpc = out.h_data.Data(:, 11);
Qin_mpc = out.h_data.Data(:, 12);

% Průtoky
% PID
Q12_ref_pid = out.h_data.Data(:, 13);
Q12_meas_pid = out.h_data.Data(:, 14);
Qout_ref_pid = out.h_data.Data(:, 15);
Qout_meas_pid = out.h_data.Data(: ,16);

% MPC
Q12_ref_mpc = out.h_data.Data(:, 17);
Q12_meas_mpc = out.h_data.Data(:, 18);
Qout_ref_mpc = out.h_data.Data(:, 19);
Qout_meas_mpc = out.h_data.Data(:, 20);

%% Grafy

% % Graf 1: Vstupní žádané signály h1 a h2
% figure('Position', [100, 100, 800, 400], 'Color', 'w');
% plot(t, h1_sp, 'b-', 'LineWidth', 2); hold on;
% plot(t, h2_sp, 'r-', 'LineWidth', 2);
% title('Graf 1: Žádané hodnoty pro hladiny h_1 a h_2');
% xlabel('Čas [s]'); 
% ylabel('Žádaná výška hladiny [m]');
% legend('h_{1,SP} (Nádrž 1)', 'h_{2,SP} (Nádrž 2)', 'Location', 'best');
% grid on;

% Graf 2: Srovnání ventilů (PID vs MPC)
figure('Position', [150, 150, 1200, 400], 'Color', 'w');

% Subplot 1: PID ventily
subplot(1, 2, 1);
plot(t, z1_pid, 'b-', 'LineWidth', 1.5); hold on;
plot(t, z2_pid, 'r--', 'LineWidth', 1.5);
title('PID: Akční zásahy ventilů');
xlabel('Čas [s]'); ylabel('Poloha ventilu z [-]');
legend('z_1 (PID)', 'z_2 (PID)', 'Location', 'best');
grid on;
ylim([0 1]);

% Subplot 2: MPC ventily
subplot(1, 2, 2);
plot(t, z1_mpc, 'b-', 'LineWidth', 1.5); hold on;
plot(t, z2_mpc, 'r--', 'LineWidth', 1.5);
title('MPC: Akční zásahy ventilů');
xlabel('Čas [s]'); ylabel('Poloha ventilu z [-]');
legend('z_1 (MPC)', 'z_2 (MPC)', 'Location', 'best');
grid on;
ylim([0 1]);

% % Graf 3: Srovnání skutečných hladin bez setpointu (PID vs MPC)
% figure('Position', [200, 200, 1200, 400], 'Color', 'w');
% 
% % Subplot 1: PID hladiny
% subplot(1, 2, 1);
% plot(t, h1_pid, 'b-', 'LineWidth', 1.5); hold on;
% plot(t, h2_pid, 'r-', 'LineWidth', 1.5);
% title('PID: Odezva skutečných hladin');
% xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
% legend('h_1 (PID)', 'h_2 (PID)', 'Location', 'best');
% grid on;
% 
% % Subplot 2: MPC hladiny
% subplot(1, 2, 2);
% plot(t, h1_mpc, 'b-', 'LineWidth', 1.5); hold on;
% plot(t, h2_mpc, 'r-', 'LineWidth', 1.5);
% title('MPC: Odezva skutečných hladin');
% xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
% legend('h_1 (MPC)', 'h_2 (MPC)', 'Location', 'best');
% grid on;

% Graf 4: Srovnání teorie a reality
figure('Position', [250, 250, 1200, 500], 'Color', 'w');

% Subplot 1: PID
subplot(1, 2, 1);
plot(t, h1_sp, 'k--', 'LineWidth', 1.5); hold on;
plot(t, h1_pid, 'b-', 'LineWidth', 1.5);
plot(t, h2_sp, 'k:', 'LineWidth', 2);
plot(t, h2_pid, 'r-', 'LineWidth', 1.5);
title('PID: Průběh žádané hodnoty');
xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
legend('h_{1,SP} (Teorie)', 'h_1 (PID)', 'h_{2,SP} (Teorie)', 'h_2 (PID)', 'Location', 'best');
grid on;

% Subplot 2: MPC
subplot(1, 2, 2);
plot(t, h1_sp, 'k--', 'LineWidth', 1.5); hold on;
plot(t, h1_mpc, 'b-', 'LineWidth', 1.5);
plot(t, h2_sp, 'k:', 'LineWidth', 2);
plot(t, h2_mpc, 'r-', 'LineWidth', 1.5);
title('MPC: Průběh žádané hodnoty');
xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
legend('h_{1,SP} (Teorie)', 'h_1 (MPC)', 'h_{2,SP} (Teorie)', 'h_2 (MPC)', 'Location', 'best');
grid on;

% Graf 5: Přímé srovnání PID a MPC pro jednotlivé nádrže
figure('Position', [300, 300, 1200, 450], 'Color', 'w');

% Subplot 1: Srovnání pro h1
subplot(1, 2, 1);
plot(t, h1_sp, 'k--', 'LineWidth', 1.5); hold on;
plot(t, h1_pid, 'b-', 'LineWidth', 1.5);
plot(t, h1_mpc, 'r-', 'LineWidth', 1.5);
title('Srovnání řízení hladiny h_1 (Nádrž 1)');
xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
legend('Žádaná hodnota (h_{1,SP})', 'PID', 'MPC', 'Location', 'best');
grid on;

% Subplot 2: Srovnání pro h2
subplot(1, 2, 2);
plot(t, h2_sp, 'k--', 'LineWidth', 1.5); hold on;
plot(t, h2_pid, 'b-', 'LineWidth', 1.5);
plot(t, h2_mpc, 'r-', 'LineWidth', 1.5);
title('Srovnání řízení hladiny h_2 (Nádrž 2)');
xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
legend('Žádaná hodnota (h_{2,SP})', 'PID', 'MPC', 'Location', 'best');
grid on;

% Graf 6: Přímé srovnání akčních zásahů pro jednotlivé ventily

figure('Position', [350, 350, 1200, 450], 'Color', 'w');

% Subplot 1: Srovnání pro z1
subplot(1, 2, 1);
plot(t, z1_pid, 'b-', 'LineWidth', 1.5); hold on;
plot(t, z1_mpc, 'r-', 'LineWidth', 1.5);
title('Srovnání otevření ventilu z_1 (Nádrž 1)');
xlabel('Čas [s]'); ylabel('Poloha ventilu [-]');
legend('PID', 'MPC', 'Location', 'best');
grid on;
ylim([0 1]);

% Subplot 2: Srovnání pro z2
subplot(1, 2, 2);
plot(t, z2_pid, 'b-', 'LineWidth', 1.5); hold on;
plot(t, z2_mpc, 'r-', 'LineWidth', 1.5);
title('Srovnání otevření ventilu z_2 (Nádrž 2)');
xlabel('Čas [s]'); ylabel('Poloha ventilu [-]');
legend('PID', 'MPC', 'Location', 'best');
grid on;
ylim([0 1]);

% Graf 7: Průtoky - žádaný vs. skutečný
figure('Position', [400, 100, 1200, 800], 'Color', 'w');

% 1. PID Nádrž 1
subplot(2, 2, 1);
plot(t, Q12_ref_pid, 'k--', 'LineWidth', 1.5); hold on;
plot(t, Q12_meas_pid, 'b-', 'LineWidth', 1.5);
title('PID kaskáda: Průtok Q_{12} (Nádrž 1)');
xlabel('Čas [s]'); ylabel('Průtok [m^3/s]');
legend('Žádaný průtok (LC1)', 'Skutečný průtok', 'Location', 'best');
grid on;

% 2. PID Nádrž 2
subplot(2, 2, 2);
plot(t, Qout_ref_pid, 'k--', 'LineWidth', 1.5); hold on;
plot(t, Qout_meas_pid, 'b-', 'LineWidth', 1.5);
title('PID kaskáda: Průtok Q_{out} (Nádrž 2)');
xlabel('Čas [s]'); ylabel('Průtok [m^3/s]');
legend('Žádaný průtok (LC2)', 'Skutečný průtok', 'Location', 'best');
grid on;

% 3. MPC Nádrž 1
subplot(2, 2, 3);
plot(t, Q12_ref_mpc, 'k--', 'LineWidth', 1.5); hold on;
plot(t, Q12_meas_mpc, 'r-', 'LineWidth', 1.5);
title('MPC: Průtok Q_{12} (Nádrž 1)');
xlabel('Čas [s]'); ylabel('Průtok [m^3/s]');
legend('Žádaný průtok (MPC)', 'Skutečný průtok', 'Location', 'best');
grid on;

% 4. MPC Nádrž 2
subplot(2, 2, 4);
plot(t, Qout_ref_mpc, 'k--', 'LineWidth', 1.5); hold on;
plot(t, Qout_meas_mpc, 'r-', 'LineWidth', 1.5);
title('MPC: Průtok Q_{out} (Nádrž 2)');
xlabel('Čas [s]'); ylabel('Průtok [m^3/s]');
legend('Žádaný průtok (MPC)', 'Skutečný průtok', 'Location', 'best');
grid on;