classdef GPC_Controller < handle
    properties
        N           % Predikční horizont
        Nu          % Horizont řízení
        lambda      % Penalizace akčního zásahu
        
        A_tilde     % Rozšířený jmenovatel A(z^-1)*Delta
        B           % Čitatel B(z^-1)
        
        G           % Matice dynamiky (vnucená odezva)
        H       % Inverze Hessovy matice pro QP
        
        % Vnitřní paměť historie signálů
        y_hist
        du_hist
        u_prev      % Minulý absolutní akční zásah u(t-1)
    end
    
    methods
        function obj = GPC_Controller(A, B, N, Nu, lambda)
            obj.N = N;
            obj.Nu = Nu;
            obj.lambda = lambda;
            
            % 1. CARIMA polynom A_tilde = A * (1 - z^-1) [3]
            obj.A_tilde = conv(A, [1, -1]);
            obj.B = B;
            
            % 2. Sestavení dynamické matice G pomocí přechodové charakteristiky [5]
            step_resp = filter(B, obj.A_tilde, ones(N, 1)); % odezva na jedn. impulz
            for i = 1:Nu
                obj.G(i:N, i) = step_resp(1:N-i+1);
                % disp(obj.G);
            end
            
            % 3. Příprava matice H pro Kvadratické programování [6]
            % H = 2 * (G^T * G + lambda * I) > 0
            % (w - y)^T * (w - y) + lambda * delta_u^T * delta_u
            obj.H = 2 * (obj.G' * obj.G + lambda * eye(Nu));
            
            % Inicializace paměti
            obj.y_hist = zeros(length(obj.A_tilde)-1, 1);
            obj.du_hist = zeros(length(obj.B)-1, 1);
            obj.u_prev = 0;
        end
        
        function u = update(obj, w_val, y_curr, u_min, u_max, du_min, du_max)
            % Aktualizace historie výstupu
            obj.y_hist = [y_curr; obj.y_hist(1:end-1)];
            
            % 1. VÝPOČET VOLNÉ ODEZVY f (předpoklad budoucí du = 0) [5]
            f = zeros(obj.N, 1);
            y_tmp = obj.y_hist;
            du_tmp = obj.du_hist;
            
            for k = 1:obj.N
                y_next = 0;
                % Vliv setrvacnosti vystupu (A_tilde)
                for i = 2:length(obj.A_tilde)
                    if (k - i + 1) > 0 % sahame do budoucnosti -> z hodnot vypocit. f
                        y_next = y_next - obj.A_tilde(i) * f(k - i + 1);
                    else
                        % sahame do minulosti -> z hodnot y_tmp
                        y_next = y_next - obj.A_tilde(i) * y_tmp(-(k - i + 1) + 1);
                    end
                end
                % Vliv minulych zasahu vstupu (polynom B) -> vsechny
                % budouci zmeny = 0 -> neni budoucnost
                for i = 2:length(obj.B)
                    if (k - i + 1) <= 0
                        y_next = y_next + obj.B(i) * du_tmp(-(k - i + 1) + 1);
                    end
                end
                f(k) = y_next; % chybovy vektor -> w - f
            end
            
            % 2. FORMULACE QP [6]
            w_vec = w_val * ones(obj.N, 1);
            b_qp = 2 * obj.G' * (f - w_vec); % Lineární člen vektor b
            
            % 3. ZAVEDENÍ OMEZENÍ (Matice A_ineq * du <= b_ineq) [7]
            I = eye(obj.Nu); % jednotkovy vektor
            T = tril(ones(obj.Nu)); % dolni troj. matice funguje jako diskretni 
                                    % integrator
            
            A_ineq = [I; -I; T; -T]; % Rozhodovaci matice omezeni
            b_ineq = [du_max * ones(obj.Nu, 1); % Vektor limitu
                     -du_min * ones(obj.Nu, 1);
                     (u_max - obj.u_prev) * ones(obj.Nu, 1);
                     (-u_min + obj.u_prev) * ones(obj.Nu, 1)];
            
            % 4. OPTIMALIZACE POMOCÍ QUADPROG
            % Syntaxe: x = quadprog(H, f, A, b)
            du_opt = quadprog(obj.H, b_qp, A_ineq, b_ineq);
            
            % Princip ustupujícího horizontu - bereme jen první zásah [6]
            du_k = du_opt(1);
            
            % 5. AKTUALIZACE A ODESLÁNÍ AKČNÍHO ZÁSAHU
            obj.u_prev = obj.u_prev + du_k;
            obj.du_hist = [du_k; obj.du_hist(1:end-1)];
            
            u = obj.u_prev;
        end
    end
end
