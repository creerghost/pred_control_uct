 function main(r0, Ti, Td)
    clc; close all;
    
    if nargin < 3
        r0 = 1.2;
        Ti = 3.7;
        Td = 0.5;
    end
    
    s = tf('s');
    Fs = 1.5 / (5*s^2 + 5*s + 1);
    
    T = 0.5;
    N = 120;
    w = ones(N, 1);
    y = zeros(N, 1);
    u = zeros(N, 1);
    
    % Diskretizace systému
    Fz = c2d(Fs, T, 'zoh'); % Zero-Order Hold
    [num, den] = tfdata(Fz, 'v');
    
    a1 = den(2); a2 = den(3);
    b1 = num(2); b2 = num(3);
    
    % Ladění PID
    
    % vytvoření objektu
    pid = DiscretePID(r0, Ti, Td, T);
    
    % Simulační smýčka
    for k = 3:N
        y(k) = -a1*y(k-1) - a2*y(k-2) + b1*u(k-1) + b2*u(k-2);
        u(k) = pid.update(w(k), y(k));
    end
    
    % Grafy
    time = (0:N-1) * T;
    plot(time, y, 'b', 'LineWidth', 1.5); hold on;
    plot(time, w, 'r--', 'LineWidth', 1.5);
    title('Odezva PID regulátoru');
    xlabel('Čas [s]'); ylabel('y(k)');
    legend('Výstup', 'Reference', 'Location', 'best');
    ylim([0 1.2])
    grid on;
end