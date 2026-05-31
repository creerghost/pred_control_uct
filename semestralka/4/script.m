clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;                        % Plocha dna nádrží [m^2]
S = 25 * 1e-4;                % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;              % Vstupní průtok [m^3/s]
T_sim = 60000;
Qin_step = 50;
A_inv = 1 / A;
k_valve = S * sqrt(2 * g);
f_sin = 2*pi/(T_sim / 8);     % frekvence kmitání sinosového signálu
Tv = 0.1;                     % časová konstanta pro dynamiku ventilů
h_sp = 0.8155;

% Parametry regulačních ventilů (Úkol 3)
z1_0 = 0.5;
z2_0 = 0.5;

% Výpočet ustálených stavů (zahrnující ventily)
h1_calced = (Qin / (z1_0 * k_valve))^2;
h2_calced = (Qin / (z2_0 * k_valve))^2;

% zlomky přenosové funkce ventilů
num = 1;
den = [Tv 1];

% setpoint hladiny
num_hl = [0 T_sim/2 T_sim];
den_hl = [0 1 0];

% Vnější smyčky (Level Control)
Kp_LC1 = -10; Ki_LC1 = -1;
Kp_LC2 = Kp_LC1; Ki_LC2 = Ki_LC1;
% Vnitřní smyčky (Flow Control)
Kp_FC1 = 10;   Ki_FC1 = 1;
Kp_FC2 = Kp_FC1;   Ki_FC2 = Ki_FC1;

%% Simulink

% Spuštění modelu Simulink
model_name = 'model';

% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

% Extrakce dat
t = out.h_data.Time;
z1 = out.h_data.Data(:, 1);
z2 = out.h_data.Data(:, 2);
h1_nl = out.h_data.Data(:, 3);
h2_nl = out.h_data.Data(:, 4);
Qin_sim = out.h_data.Data(:, 5);


%% Grafy

% Graf 1
figure('Position', [100, 100, 800, 400], 'Color', 'w');
plot(t, z1, 'b-', 'LineWidth', 1.5); hold on;
plot(t, z2, 'r--', 'LineWidth', 1.5);

title('Otevření a zavření regulačních ventilů v čase');
xlabel('Čas [s]');
ylabel('Poloha ventilu z [-]');
legend('z_1', 'z_2', 'Location', 'best');
grid on;
ylim([-0.1 1.1]);

% Graf 2
figure('Position', [150, 150, 1400, 400], 'Color', 'w');

% Graf 2.1: Vstupní signál
subplot(1, 3, 1);
plot(t, Qin_sim, 'm-', 'LineWidth', 1.5);
title('Vstupní průtok do kaskády');
xlabel('Čas [s]'); 
ylabel('Průtok Q_{in} [m^3/s]');
grid on; 

% Graf 2.2: Nádrž 1
subplot(1, 3, 2);
plot(t, h1_nl, 'b-', 'LineWidth', 1.5); hold on;
yline(h_sp, 'k--', 'Žádaná hodnota', 'LineWidth', 1.5);
title('Nádrž 1');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_1 [m]');
legend('Skutečná hladina (h_1)', 'Žádaná hodnota (h_{SP})', 'Location', 'best'); 
grid on;

% Graf 2.3: Nádrž 2
subplot(1, 3, 3);
plot(t, h2_nl, 'r-', 'LineWidth', 1.5); hold on;
yline(h_sp, 'k--', 'Žádaná hodnota', 'LineWidth', 1.5);
title('Nádrž 2');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_2 [m]');
legend('Skutečná hladina (h_2)', 'Žádaná hodnota (h_{SP})', 'Location', 'best'); 
grid on;