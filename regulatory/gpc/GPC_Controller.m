classdef GPC_Controller < handle
    properties
        N % Predikční horizont
        Nu % Horizont řízení
        lambda % Penalizace akčního zásahu
        
        A_tilde % Rozšířený jmenovatel A(z^-1)*Delta
        B % Čitatel B(z^-1)
        
        G % Matice dynamiky (vnucená odezva)
        H % Hessova matice pro QP
        
        % Vnitřní paměť historie signálů
        y_hist
        du_hist
        u_prev
        
        % Matice historie pro volnou odezvu
        F_y
        F_u
    end
    % Methods
    methods
        function obj = GPC_Controller(A, B, N, Nu, lambda)
            obj.N = N;
            obj.Nu = Nu;
            obj.lambda = lambda;
            
            %% 1. CARIMA polynom A_tilde = A * (1 - z^-1)
            obj.A_tilde = conv(A, [1, -1]);
            obj.B = B;
            
            % y = Gu + Fh = nucená + volná odezva = budoucí zásahy ventilu
            % + reakce systemu na minulé akční zásahy bez ovlivnení nových
            % Pro získání predikčního modelu byla použita metoda inverzní
            % matice, je velmi dobře popsána v přednášce
            
            %% 2. Sestavení matice A_mat pro budoucí sestavení matice G
            % Dolní trojúhelníková matice s jedničkami na diagonále a koeficienty A_tilde pod ní
            A_mat = eye(obj.N);
            for i = 2:length(obj.A_tilde)
                % Funkce diag nasází koeficienty na příslušnou sub-diagonálu
                A_mat = A_mat + diag(obj.A_tilde(i) * ones(obj.N - i + 1, 1), -(i - 1));
            end
            
            %% 3. Sestavení matice B_mat pro budoucí sestavení matice G
            % Dolní trojúhelníková matice s koeficienty polynomu B
            B_mat = zeros(obj.N, obj.N);
            for i = 2:length(obj.B)
                % Index -(i-2) zajistí, že hodnota b1 (tedy B(2)) padne přesně na hlavní diagonálu
                B_mat = B_mat + diag(obj.B(i) * ones(obj.N - i + 2, 1), -(i - 2));
            end
            
            %% 4. Výpočet matice dynamiky G
            % G_full = inv(A_mat) * B_mat (maticové dělení)
            G_full = A_mat \ B_mat;
            
            %% 5. Oříznutí matice G pouze na horizont řízení Nu
            obj.G = G_full(:, 1:obj.Nu);
            
            % Matice má koeficienty přechodové charakteristiky. První
            % sloupec je reakce na skok a další jsou to samé, ale posunuté
            % v čase
            
            %% 6. Příprava matice H pro QP
            
            % Hessova matice = matice druhých derivací
            % Nemění se v čase, a proto můžeme jí vypočíst dřívě

            % H = 2 * (G^T * G + lambda * I) > 0
            % (w - y)^T * (w - y) + lambda * delta_u^T * delta_u
            obj.H = 2 * (obj.G' * obj.G + lambda * eye(Nu));
            
            %% 7. Příprava matic historie pro volnou odezvu
            m = length(obj.A_tilde) - 1;
            n_du = length(obj.B) - 1;
            
            % Sestavení matice tilde_A (vliv minulých výstupů)
            t_A = zeros(obj.N, m);
            a_poly = obj.A_tilde(2:end);
            for i = 1:obj.N
                for j = 1:m
                    idx = (i-1) + (j-1);
                    if idx < m
                        t_A(i, j) = -a_poly(idx + 1);
                    end
                end
            end
            
            % Sestavení matice tilde_B (vliv minulých vstupů)
            t_B = zeros(obj.N, n_du);
            for i = 1:obj.N
                for j = 1:n_du
                    i_loop = j + i;
                    if i_loop <= length(obj.B)
                        t_B(i, j) = obj.B(i_loop);
                    end
                end
            end
            
            % Výpočet finálních matic historie přes inverzní matici
            obj.F_y = A_mat \ t_A;
            obj.F_u = A_mat \ t_B;
            
            % Inicializace paměti
            obj.y_hist = zeros(length(obj.A_tilde)-1, 1);
            obj.du_hist = zeros(length(obj.B)-1, 1);
            obj.u_prev = 0;
        end
        % Metoda update - volá se každou iteraci
        function u = update(obj, w_val, y_curr, use_constraints, u_min, u_max, du_min, du_max)
            % w_val - žádaná hodnota
            % y_curr - aktuální výstup
            % use_constraints - flag pro zápnutí omezujících podmínek
            % u_min, u_max, du_min, du_max - omezující podmínky

            % Aktualizace historie výstupu
            obj.y_hist = [y_curr; obj.y_hist(1:end-1)];
            

            %% 1. Výpočet volné odezvy f
            f = obj.F_y * obj.y_hist + obj.F_u * obj.du_hist;
            
            
            %% 2. Formulace QP

            % Optimalizujeme účelovou funkci, která minimalizuje kvadrat
            % odchýlky J = (w - y)^T * (w - y) + lambda * du^T * du
            w_vec = w_val * ones(obj.N, 1);
            b_qp = 2 * obj.G' * (f - w_vec); % lineární koeficient vektor b
            

            %% 3. Zavedení omezení (Matice A_ineq * du <= b_ineq)
            I = eye(obj.Nu); % Jednotkový vektor (rychlost)
            T = tril(ones(obj.Nu)); % Dolni troj. matice (poloha)

            % u+min <= U <= u+max => známe jenom du =>
            % U(k) = U(k - 1) + dU(k) + dU(k + 1) => dolní troj. matice

            % 4 rovnice: 
            % I * dU <= du_max
            % -I * dU <= -du_min
            % T * dU <= u_max - u_prev
            % -T * dU <= -u_min + u_prev

            A_ineq = [I; -I; T; -T]; % Rozhodovací matice omezení

            b_ineq = [du_max * ones(obj.Nu, 1); % Vektor limitů
                     -du_min * ones(obj.Nu, 1);
                     (u_max - obj.u_prev) * ones(obj.Nu, 1);
                     (-u_min + obj.u_prev) * ones(obj.Nu, 1)];
            

            %% 4. Optimalizace
            if use_constraints
                % Převod notace pro lepší pochopení
                % Notace jsou z přednášky 8
                % min {p^T * x + x^T * C * x}
                % J = 1/2 * dU^T * H * dU + b_qp^T * dU
                % C = 1/2 * H, p = b_qp
                C = 0.5 * obj.H;
                p = b_qp;
                A = A_ineq;
                b = b_ineq;
                
                du_opt = obj.hildreth(C, p, A, b);
            else
                % Neomezené analytické řešení
                du_opt = -obj.H \ b_qp;
            end
            
            % Princip ustupujícího horizontu => bereme jen první zásah
            du_k = du_opt(1);
            
            %% 5. Aktualizace a odeslání akč. zásahu
            obj.u_prev = obj.u_prev + du_k;
            obj.du_hist = [du_k; obj.du_hist(1:end-1)];
            
            u = obj.u_prev;
        end
    end
    %% Skryté metody
    methods (Access = private)
        function x_opt = hildreth(obj, C, p, A, b)
            % Počítáme dU, ale máme spoustu podmínek
            % Pro počítač je to noční můra, a proto chceme to řešit v tzv.
            % duálním prostoru. V tomto prostoru hledáme pouze to, jak
            % velké mají být pokuty u, které nesmí být záporné
            
            % Hildreth-D'Esopova metoda
            % Minimalizuje: p^T * x + x^T * C * x
            % Za podmínek:  A * x <= b
            
            %% 1. Výpočet neomezeného řešení
            % x* = -1/2 C^-1 * p
            x_free = -0.5 * (C \ p);
            
            % Pokud volné řešení splňuje všechna omezení, můžeme ho rovnou použít
            % (přidána malá tolerance 1e-8 pro numerickou stabilitu)
            if all(A * x_free <= b + 1e-8)
                x_opt = x_free;
                return;
            end
            
            %% 2. Překlopení problému do duálního prostoru
            % G = 1/4 A C^-1 A^T
            % h = 1/2 A C^-1 p + b
            G = 0.25 * A * (C \ A');
            h = 0.5 * A * (C \ p) + b;
            
            m = size(h, 1);
            u = zeros(m, 1);
            u_p = zeros(m, 1);
            
            %% 3. Iterační výpočet pro duální proměnné u
            max_iter = 100;
            for k = 1:max_iter
                for i = 1:m
                    % Součet všech prvků v řádku kromě diagonály
                    sum_Gu = G(i,:) * u - G(i,i) * u(i);
                    
                    % Výpočet nového kandidáta (w) a ořezání na nezáporná čísla
                    w = -1 / G(i,i) * (sum_Gu + 0.5 * h(i));
                    u(i) = max(0, w);
                end
                
                %% 4. Ustálili se hodnoty?
                if norm(u - u_p) < 1e-6
                    break;
                end
                u_p = u;
            end
            
            %% 5. Zpětný převod z duálního do primárního prostoru
            % x* = -1/2 C^-1 (A^T u^* + p)
            x_opt = -0.5 * (C \ (A' * u + p));
        end
    end
end
