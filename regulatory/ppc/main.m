clc; close all;

s = tf('s');
Fs = 1.5 / (5*s^2 + 5*s + 1);
T = 0.5;

Fz = c2d(Fs, T, 'zoh');
[num, den] = tfdata(Fz, 'v');
B = num; % 2. řád
A = den; % 2. řád
% Návrh žádaného charakteristického polynomu D
% Protože na=2 a nb=2, Sylvestrova matice má velikost 2 + 2 = 4 (4x4). 
% Náš žádaný polynom D tedy musí mít 2. řád (musíme zadat 3 póly).
% Zvolíme stabilní póly uvnitř jednotkové kružnice (Z-oblast):
poly_eig = [0.5, 0.5, 0.5];
D = poly(poly_eig); % Vytvoří koeficienty polynomu D

% Inicializace PPC
ppc = PPC(A, B, D);

% Nastavení parametrů simulace
N = 80;
w = ones(N, 1); % skok
y = zeros(N, 1);
u = zeros(N, 1);

% Koeficienty pro ruční simulaci soustavy
a1 = A(2);
a2 = A(3);
b1 = B(2);
b2 = B(3);

% Simulační smyčka
for k = 3:N
    y(k) = -a1*y(k-1) - a2*y(k-2) + b1*u(k-1) + b2*u(k-2);
    u(k) = ppc.update(w(k), y(k));
end

time = (0:N-1) * T;
plot(time, y, 'b-', 'LineWidth', 2); hold on;
plot(time, w, 'r--', 'LineWidth', 1.5);
title('Odezva PPC regulátoru');
legend('Výstup', 'Reference', 'Location', 'best'); grid on;
ylim([0 1.2])