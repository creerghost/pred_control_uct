clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;              % Plocha dna nádrží [m^2]
S = 25 * 1e-4;      % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;    % Vstupní průtok [m^3/s]
T_sim = 60000;
A_inv = 1 / A;
k_valve = S * sqrt(2 * g);
T_step = 30000;
Tv = 0.1; % časová konstanta pro dynamiku ventilů

% Parametry regulačních ventilů (Úkol 3)
z1_0 = 0.5;           % Ustálená poloha ventilu 1 (50 % otevření)
z2_0 = z1_0;           % Ustálená poloha ventilu 2 (50 % otevření)

% Výpočet ustálených stavů (zahrnující ventily)
h1_calced = (Qin / (z1_0 * k_valve))^2;
h2_calced = (Qin / (z2_0 * k_valve))^2;

% zlomky přenosové funkce ventilů
num = 1;
den = [Tv 1];

%% Simulink

% Spuštění modelu Simulink
model_name = 'model';
% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

% Extrakce dat
t = out.h_data.Time;
h1 = out.h_data.Data(:, 1);
h2 = out.h_data.Data(:, 2);
Qin_sim = out.h_data.Data(:, 3);

%% Grafy
figure('Position', [100, 100, 1400, 400], 'Color', 'w');

% Graf 1
subplot(1, 3, 1);
plot(t, Qin_sim, 'k-', 'LineWidth', 1.5);
title('Skokový signál');
xlabel('Čas [s]'); 
ylabel('Průtok [m^3/s]');
grid on; ylim([0 Qin*1.2])

% Graf 2
subplot(1, 3, 2);
plot(t, h1, 'k-', 'LineWidth', 1.5); hold on;
yline(h1_calced, 'r--', 'Vypočtený ustálený stav', 'LineWidth', 1.5);
title('Odezva nelineárního modelu - Nádrž 1');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_1 [m]');
legend('Nelineární model', 'Analytický výpočet', 'Location', 'southeast'); 
grid on; ylim([0 4])

% Graf 3
subplot(1, 3, 3);
plot(t, h2, 'k-', 'LineWidth', 1.5); hold on;
yline(h2_calced, 'r--', 'Vypočtený ustálený stav', 'LineWidth', 1.5);
title('Odezva nelineárního modelu - Nádrž 2');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_2 [m]');
legend('Nelineární model', 'Analytický výpočet', 'Location', 'southeast'); 
grid on; ylim([0 4])