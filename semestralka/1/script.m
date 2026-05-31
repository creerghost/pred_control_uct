clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;              % Plocha dna nádrží [m^2
S = 25 * 1e-4;      % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;    % Vstupní průtok [m^3/s]
T_sim = 10000;

% Inicializace vzorců pro Simulink
A_inv = 1/ A;
k_valve = S * sqrt(2 * g);

%% Simulink

% Spuštění modelu Simulink
model_name = 'model';

% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

% Extrakce dat
t = out.h_data.Time;           % Časový vektor
h1 = out.h_data.Data(:, 1);    % 1. vstup
h2 = out.h_data.Data(:, 2);    % 2. vstup

%% Graf

% Vykreslení grafu
plot(t, h1, 'b-', 'LineWidth', 2); hold on;
plot(t, h2, 'r--', 'LineWidth', 2);

% Přidání analytického výpočtu
h_calced = (Qin / S)^2 / (2 * g);
yline(h_calced, 'k:', 'Vypočtený ustálený stav', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');

% Formátování grafu
title('Plnění nádrží');
xlabel('Čas [s]'); ylabel('Výška hladiny [m]');
legend('h_1', 'h_2'); grid on;
xlim([0 T_sim]); ylim([0 1]);
