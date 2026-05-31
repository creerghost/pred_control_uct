clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;              % Plocha dna nádrží [m^2]
S = 25 * 1e-4;      % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;    % Vstupní průtok [m^3/s]
T_sim = 50000;
Qin_step = 50;
A_inv = 1 / A;
k_valve = S * sqrt(2 * g);
T_step = 25000;
Tv = 0.1; % časová konstanta pro dynamiku ventilů

% Parametry regulačních ventilů (Úkol 3)
z1_0 = 0.5;           % Ustálená poloha ventilu 1 (50 % otevření)
z2_0 = 0.5;           % Ustálená poloha ventilu 2 (50 % otevření)

% Výpočet ustálených stavů (zahrnující ventily)
h1_calced = (Qin / (z1_0 * k_valve))^2;
h2_calced = (Qin / (z2_0 * k_valve))^2;

% zlomky přenosové funkce ventilů
num = 1;
den = [Tv 1];

% setpoint hladiny
num_hl = [0 T_sim/2 T_sim];
den_hl = [0 1 0];

%% PID regulátor
% Vnější smyčky (Level Control)
Kp_LC1 = -15; Ki_LC1 = -1.2;
Kp_LC2 = -15; Ki_LC2 = -1.2;

% Vnitřní smyčky (Flow Control)
Kp_FC1 = 5;   Ki_FC1 = 0.5;
Kp_FC2 = 5;   Ki_FC2 = 0.5;
%% Simulink

% Spuštění modelu Simulink
model_name = 'model';
% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

% Extrakce dat
t = out.h_data.Time;
h1_nl = out.h_data.Data(:, 3);
h1_sp = out.h_data.Data(:, 1);
h2_nl = out.h_data.Data(:, 4);
h2_sp = out.h_data.Data(:, 2);

%% Grafy
figure('Position', [100, 100, 1200, 450]);

% Graf 1: nádrž 1
subplot(1, 2, 1);
plot(t, h1_nl, 'b-', 'LineWidth', 2); hold on;
plot(t, h1_sp, 'k--', 'LineWidth', 1.5);

% Formátování grafu 1
title('Sledování žádané hodnoty hladiny h_1 (Nádrž 1)');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_1 [m]');
legend('Skutečná hladina (h_1)', 'Žádaná hodnota (h_1_{,SP})', 'Location', 'southeast'); 
grid on;

% Graf 2: nádrž 2
subplot(1, 2, 2);
plot(t, h2_nl, 'r-', 'LineWidth', 2); hold on;
plot(t, h2_sp, 'k--', 'LineWidth', 1.5);

% Formátování grafu 2
title('Sledování žádané hodnoty hladiny h_2 (Nádrž 2)');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_2 [m]');
legend('Skutečná hladina (h_2)', 'Žádaná hodnota (h_2_{,SP})', 'Location', 'southeast'); 
grid on;