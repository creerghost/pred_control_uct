clear; clc; close all;

addpath('gpc');
addpath('ppc');
addpath('pid');

%% 1. Definice systému a diskretizace
s = tf('s');
Fs = 1.5 / (5*s^2 + 5*s + 1);
T = 0.5;
Fz = c2d(Fs, T, 'zoh');
[num, den] = tfdata(Fz, 'v');
A = den;
B = num;

% Koeficienty pro simulaci
a1 = A(2); a2 = A(3);
b1 = B(2); b2 = B(3);

% Simulační smyčka a omezení
N_sim = 300;
w = zeros(N_sim, 1);
w(floor(N_sim/2)+1:end) = 1;

%% 2. Inicializace všech tří regulátorů

% A) PID
r0 = 1.2; Ti = 3.7; Td = 0.5;
pid_ctrl = DiscretePID(r0, Ti, Td, T);

% B) PPC
poly_eig = [0.5, 0.5, 0.5];
D = poly(poly_eig);
ppc_ctrl = PPC(A, B, D);

% C) GPC
N_gpc = 100;
Nu_gpc = 5;
lambda_gpc = 0.5;
gpc_ctrl = GPC_Controller(A, B, N_gpc, Nu_gpc, lambda_gpc);

% Fyzikální omezení akčního členu (zpracuje ho jen GPC)
use_constraints_gpc = true; % Přepínač pro zapnutí/vypnutí omezení u GPC
u_min = 0;   u_max = 1;
du_min = -0.05; du_max = 0.2;

%% 3. Paměti pro záznam průběhů
y_pid = zeros(N_sim, 1); u_pid = zeros(N_sim, 1);
y_ppc = zeros(N_sim, 1); u_ppc = zeros(N_sim, 1);
y_gpc = zeros(N_sim, 1); u_gpc = zeros(N_sim, 1);

%% 4. Hlavní simulační smýčka
for k = 3:N_sim
    % A) PID Větev
    y_pid(k) = -a1*y_pid(k-1) - a2*y_pid(k-2) + b1*u_pid(k-1) + b2*u_pid(k-2);
    u_pid(k) = pid_ctrl.update(w(k), y_pid(k));
    
    % B) PPC Větev
    y_ppc(k) = -a1*y_ppc(k-1) - a2*y_ppc(k-2) + b1*u_ppc(k-1) + b2*u_ppc(k-2);
    u_ppc(k) = ppc_ctrl.update(w(k), y_ppc(k));
    
    % C) GPC Větev
    y_gpc(k) = -a1*y_gpc(k-1) - a2*y_gpc(k-2) + b1*u_gpc(k-1) + b2*u_gpc(k-2);
    u_gpc(k) = gpc_ctrl.update(w(k), y_gpc(k), use_constraints_gpc, u_min, u_max, du_min, du_max);
end

%% 5. Grafy
time = (0:N_sim-1) * T;
figure('Name', 'Porovnání Regulátorů (PID vs PPC vs GPC)', 'Position', [100 100 800 600]);

% Horní graf - Odezvy (Výstupy y)
subplot(2,1,1);
plot(time, w, 'k--', 'LineWidth', 1.5); hold on;
plot(time, y_pid, 'g-', 'LineWidth', 1.5);
plot(time, y_ppc, 'r-', 'LineWidth', 1.5);
plot(time, y_gpc, 'b-', 'LineWidth', 2);
title('Srovnání regulátorů - Výstupy y(k)');
ylabel('Výstup y');
legend('Reference w(k)', 'PID', 'PPC', 'GPC', 'Location', 'best');
ylim([-0.1 1.4]);
grid on;

% Dolní graf - Akční zásahy (u)
subplot(2,1,2);
stairs(time, u_pid, 'g-', 'LineWidth', 1.5); hold on;
stairs(time, u_ppc, 'r-', 'LineWidth', 1.5);
stairs(time, u_gpc, 'b-', 'LineWidth', 2);
title('Srovnání regulátorů - Akční zásahy u(k)');
xlabel('Čas [s]'); ylabel('Akční zásah u');
legend('PID', 'PPC', 'GPC', 'Location', 'best');
grid on;

%% 6. Samostatné grafy pro každý regulátor

% PID
figure('Name', 'PID Regulátor', 'Position', [150 150 600 500]);
subplot(2,1,1);
plot(time, y_pid, 'g-', 'LineWidth', 2); hold on;
plot(time, w, 'k--', 'LineWidth', 1.5);
title('Odezva PID regulátoru - Výstup');
ylabel('Výstup y(k)');
legend('Výstup', 'Reference', 'Location', 'best');
ylim([-0.1 1.4]); grid on;

subplot(2,1,2);
stairs(time, u_pid, 'g-', 'LineWidth', 1.5);
title('PID: Akční zásah u(k)');
xlabel('Čas [s]'); ylabel('Akční zásah u(k)');
grid on;

% PPC
figure('Name', 'PPC Regulátor', 'Position', [200 200 600 500]);
subplot(2,1,1);
plot(time, y_ppc, 'r-', 'LineWidth', 2); hold on;
plot(time, w, 'k--', 'LineWidth', 1.5);
title('Odezva PPC regulátoru - Výstup');
ylabel('Výstup y(k)');
legend('Výstup', 'Reference', 'Location', 'best');
ylim([-0.1 1.4]); grid on;

subplot(2,1,2);
stairs(time, u_ppc, 'r-', 'LineWidth', 1.5);
title('PPC: Akční zásah u(k)');
xlabel('Čas [s]'); ylabel('Akční zásah u(k)');
grid on;

% GPC
figure('Name', 'GPC Regulátor', 'Position', [250 250 600 500]);
subplot(2,1,1);
plot(time, y_gpc, 'b-', 'LineWidth', 2); hold on;
plot(time, w, 'k--', 'LineWidth', 1.5);
title('Odezva GPC regulátoru - Výstup');
ylabel('Výstup y(k)');
legend('Výstup', 'Reference', 'Location', 'best');
ylim([-0.1 1.4]); grid on;

subplot(2,1,2);
stairs(time, u_gpc, 'b-', 'LineWidth', 1.5);
title('GPC: Akční zásah u(k)');
xlabel('Čas [s]'); ylabel('Akční zásah u(k)');
grid on;
