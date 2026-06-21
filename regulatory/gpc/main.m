clear; clc; close all;

% Přenosová funkce a jeji diskretizace
s = tf('s');
Fs = 1.5 / (10*s^2 + 5*s + 1);
T = 0.5;
Fz = c2d(Fs, T, 'zoh');
[num, den] = tfdata(Fz, 'v');

% Polynomy modelu
A = den;
B = num;

% Parametry GPC
N = 100; % Horizont predikce
Nu = 10; % Horizont řízení
lambda = 0.5; % Penalizace změn akčního zásahu

% Fyzikální omezení akčního členu (např. ventil 0-100%, max skok 10%)
u_min = 0;   u_max = 1;
du_min = -0.1; du_max = 0.1;

% Inicializace OOP GPC
gpc = GPC_Controller(A, B, N, Nu, lambda);

% 3. Simulační parametry
N_sim = 150;
w = zeros(N_sim, 1);
w(floor(N_sim/2)+1:end) = 1; % krok začíná v polovině (po floor(N/2) vzorcích)
y = zeros(N_sim, 1);
u = zeros(N_sim, 1);

% Extrakce parametrů pro manuální simulaci objektu
a1 = A(2); a2 = A(3);
b1 = B(2); b2 = B(3);

% 4. Cyklus reálného času
for k = 3:N_sim
    % Simulace měření odezvy objektu
    y(k) = -a1*y(k-1) - a2*y(k-2) + b1*u(k-1) + b2*u(k-2);
    
    % Výpočet GPC algoritmu s uvažováním omezení (Hildreth)
    u(k) = gpc.update(w(k), y(k), u_min, u_max, du_min, du_max);
end

% 5. Vykreslení

time = (0:N_sim-1) * T;
plot(time, y, 'b-', 'LineWidth', 2); hold on;
plot(time, w, 'r--', 'LineWidth', 1.5);
title('Odezva GPC regulátoru');
legend('Výstup', 'Reference', 'Location', 'best'); grid on;
ylim([0 1.2])