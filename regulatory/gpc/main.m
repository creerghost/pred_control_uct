clear; clc; close all;

% Přenosová funkce a jeji diskretizace
s = tf('s');
Fs = 1.5 / (5*s^2 + 5*s + 1);
T = 0.5;
Fz = c2d(Fs, T, 'zoh');
[num, den] = tfdata(Fz, 'v');

% Polynomy modelu
A = den;
B = num;

% Parametry GPC
N = 50; % Horizont predikce
Nu = 5; % Horizont řízení
lambda = 0.5; % Penalizace změn akčního zásahu

% Fyzikální omezení akčního členu (např. ventil 0-100%, max skok 5%)
use_constraints = true; % Přepínač pro zapnutí/vypnutí omezení v GPC
u_min = 0;   u_max = 1;
du_min = -0.05; du_max = 0.05;

% Inicializace OOP GPC
gpc = GPC_Controller(A, B, N, Nu, lambda);

% Simulační parametry
N_sim = 150;
w = zeros(N_sim, 1);
w(floor(N_sim/2)+1:end) = 1;
y = zeros(N_sim, 1);
u = zeros(N_sim, 1);

% Koeficienty pro ruční simulaci soustavy
a1 = A(2); a2 = A(3);
b1 = B(2); b2 = B(3);

% Simulační smyčka
for k = 3:N_sim
    % Simulace měření odezvy objektu
    y(k) = -a1*y(k-1) - a2*y(k-2) + b1*u(k-1) + b2*u(k-2);
    
    % Výpočet GPC algoritmu
    u(k) = gpc.update(w(k), y(k), use_constraints, u_min, u_max, du_min, du_max);
end

% Vykreslení
figure;
time = (0:N_sim-1) * T;

subplot(2,1,1);
plot(time, y, 'b-', 'LineWidth', 2); hold on;
plot(time, w, 'r--', 'LineWidth', 1.5);
title('Odezva GPC regulátoru - Výstup');
legend('Výstup', 'Reference', 'Location', 'best'); grid on;
ylim([-0.1 1.4])

subplot(2,1,2);
stairs(time, u, 'k-', 'LineWidth', 1.5);
title('Akční zásah u(k)');
xlabel('Čas [s]'); ylabel('u(k)');
grid on;