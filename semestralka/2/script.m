clear; clc; close all;

%% Inicializace

% Inicializace proměnných
A = 4;              % Plocha dna nádrží [m^2
S = 25 * 1e-4;      % Plocha průřezu potrubí [m^2]
g = 9.81;
Qin = 10 / 1000;    % Vstupní průtok [m^3/s]
T_sim = 20000;
h_calced = (Qin / S)^2 / (2 * g);
Qin_step = 10;      % v procentech
A_inv = 1/ A;
k_valve = S * sqrt(2 * g);
T_step = 10000;

%% Linearizace

syms h1_tmp h2_tmp Qin_tmp

% NLDR (pravé strany) - dh/dt = f(h, u)
f1 = (1/A) * Qin_tmp - (k_valve/A) * sqrt(h1_tmp);
f2 = (k_valve/A) * sqrt(h1_tmp) - (k_valve/A) * sqrt(h2_tmp);

% Vektory stavů (x) a vstupů (u)
x_vars = [h1_tmp;
          h2_tmp];
u_vars = Qin_tmp;

% Matice parciálních derivací
Aj = jacobian([f1; f2], x_vars); 
Bj = jacobian([f1; f2], u_vars);

% Výpočet matic A, B
% Dosazení předem jíž známých pracovních bodů do derivací
A = double(subs(Aj, [h1_tmp, h2_tmp, Qin_tmp], [h_calced, h_calced, Qin]));
B = double(subs(Bj, [h1_tmp, h2_tmp, Qin_tmp], [h_calced, h_calced, Qin]));

% Chceme sledovat oba stavy h1, h2 => jednotková matice 
C = [1, 0; 
     0, 1];
D = [0; 
     0];

%% Příprava dat do Simulinku

% Vytvoření state-space a transfer function objektů
ss_obj = ss(A, B, C ,D);
tf_obj = tf(ss_obj);

% Export čítatelů a jmenotavelů pro použítí v Simulinku
[num1, den1] = tfdata(tf_obj(1, 1), 'v');
[num2, den2] = tfdata(tf_obj(2, 1), 'v');

%% Simulink

% Spuštění modelu Simulink
model_name = 'model';

% Funkce sim spustí model a uloží všechna data do proměnné out
out = sim(model_name, 'StopTime', num2str(T_sim));

% Extrakce dat
t = out.h_data.Time;            % Časový vektor

h1_nl = out.h_data.Data(:, 1);
h1_ss = out.h_data.Data(:, 2);
h1_tf = out.h_data.Data(:, 3);

h2_nl = out.h_data.Data(:, 4);
h2_ss = out.h_data.Data(:, 5);
h2_tf = out.h_data.Data(:, 6);

Qin_plot = out.h_data.Data(:, 7);

%% Graf
figure('Position', [100, 100, 1300, 400]);

% Graf 1
subplot(1, 3, 1); % 1 řádek, 3 sloupce, 1. pozice
plot(t, Qin_plot, 'k', 'LineWidth', 2); hold on;
title('Skokový signál');
xlabel('Čas [s]'); ylabel('Průtok [m^3/s]'); grid on; ylim([0 Qin*Qin_step/100*1.05])

% Graf 2
subplot(1, 3, 2);
plot(t, h1_nl, 'k-', 'LineWidth', 1.5); hold on;
plot(t, h1_ss, 'b--', 'LineWidth', 2);
plot(t, h1_tf, 'r:', 'LineWidth', 2);

% Formátování grafu 1
title('Odezva hladiny h_1 na skokovou změnu průtoku');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_1 [m]');
legend('Nelineární model', 'Stavový popis (SS)', 'Přenosová funkce (TF)', 'Location', 'southeast'); 
grid on;

% Graf 3
subplot(1, 3, 3);
plot(t, h2_nl, 'k-', 'LineWidth', 1.5); hold on;
plot(t, h2_ss, 'b--', 'LineWidth', 2);
plot(t, h2_tf, 'r:', 'LineWidth', 2);

% Formátování grafu 2
title('Odezva hladiny h_2 na skokovou změnu průtoku');
xlabel('Čas [s]'); 
ylabel('Výška hladiny h_2 [m]');
legend('Nelineární model', 'Stavový popis (SS)', 'Přenosová funkce (TF)', 'Location', 'southeast'); 
grid on;
