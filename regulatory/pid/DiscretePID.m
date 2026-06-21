classdef DiscretePID < handle
    % musím 'inheritovat' třídu handle. Proměnná 'pid' bude fungovat jako
    % ukazatel na přířazený objekt. Při zavolání metody update budeme
    % pracovat přímo s originálním objektem.
    properties
        % Konstanty PID
        r0, Ti, Td, T
        % Koeficienty pro odchýlkový PID
        q0, q1, q2
        % Historie
        u_prev, y_prev1, y_prev2
    end
    
    methods
        % Konstruktor
        function obj = DiscretePID(r0, Ti, Td, T)
            obj.r0 = r0;
            obj.Ti = Ti;
            obj.Td = Td;
            obj.T = T;

            obj.q0 = -r0 * (1 + T / (2 * Ti) + Td / T); % vliv aktuální chyby
            obj.q1 = r0 * (1 - T / (2 * Ti) + 2 * Td / T); % -=- minulé
            obj.q2 = -r0 * (Td / T); % -=- předminulé

            % Inicializace pamětí
            obj.u_prev = 0;
            obj.y_prev1 = 0;
            obj.y_prev2 = 0;
        end
        
        function u = update(obj, w, y)
            % Výpočet akčního zásahu (z přednásky 2)
            u = obj.u_prev - (obj.q0 + obj.q1 + obj.q2) * w ...
                + obj.q0 * y + obj.q1 * obj.y_prev1 + obj.q2 * obj.y_prev2;
            
            % Posun historie pro další časový krok
            obj.u_prev = u;
            obj.y_prev2 = obj.y_prev1;
            obj.y_prev1 = y;
        end
    end
end