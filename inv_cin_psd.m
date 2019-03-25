%% Algoritmo per l'inversione cinematica con la pseudo-inversa
% in uscita da questo algoritmo ho la Qddot ossia la velocità del giunto 

function Qddot = inv_cin_psd(Q,XYd,XYddot,a,joint_lim)

     % calcolo la posa. Successivamente valuto l'errore ossia la differenza
     % tra la posa calcolata con la function cinematica diretta e quella
     % desiderata che otteniamo dalla traiettoria.
     [~, ~, ~, XY4] = kin_man_rid_progetto(Q,a);
     % dove XY1 è un vettore 3x1 che contiene posizione x, y e orientamento
     % del giunto 1, XY2 del giubnto 2 e cosi via. Quindi XY4 contiene
     % posizione e orientamento dell'organo terminale. 

     %errore
     e = XYd - XY4;
     Ja = J_man_plan_4DoF(Q,a);
     
     % calcolo della pseudo inversa
        J_pi = Ja'/(Ja*(Ja'));
        P = eye(4) - J_pi*Ja;
             
        K=diag([1, 1, 1])*10;
                    
        if det(Ja*Ja') == 0 % quindi mi trovo in singolarità cinematica 
            Ka = diag([1, 1, 1, 1]);
            
            dw1 = funzionale_dw1(Q,a);
            
            Qddot = J_pi * (XYddot + K*e) + P*Ka*dw1; % dove Xddot è la velocità desiderata dell'organo terminale
            
        elseif ( abs(Q(1)-joint_lim(1,1)) < 1e-3 || abs(Q(1)-joint_lim(1,2)) < 1e-3...
                    || abs(Q(2)-joint_lim(2,1)) < 1e-3 || abs(Q(2)-joint_lim(2,2)) < 1e-3...
                    || abs(Q(3)-joint_lim(3,1)) < 1e-3 || abs(Q(3)-joint_lim(3,2)) < 1e-3...
                    || abs(Q(4)-joint_lim(4,1)) < 1e-3 || abs(Q(4)-joint_lim(4,2)) < 1e-3) % quindi
                % se uno qualsiasi dei giunti si trova vicino al limite di giunto
             Ka_2 = diag([1, 1, 20, 1]);
             
             dw2 = funzionale_dw2(Q,joint_lim);
             
             Qddot = J_pi * (XYddot + K*e) + P*Ka_2*dw2; 
             
        else           
            Qddot = J_pi*(XYddot+K*e);
        end
    
    end
