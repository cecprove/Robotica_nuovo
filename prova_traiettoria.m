function prova_traiettoria(traiettoria, link, t, dt)
    
    [joint_lim, ~] = inizializza_limiti;
    
    traiettoriaDot = [zeros(1,3); diff(traiettoria,1,1) / dt];
    % fissiamo un q4 e sfruttiamo la funzione cinematica_inversa_4DoF che
    % abbiamo usato per la traiettoria. Tanto questo passaggio ci serve solo
    % per calcolare le posizioni iniziali con cui far partire il manipolatore,
    % successivamnete implementiamo l'algoritmo di inversione cinematica.
    % Scelgo arbitrariamente q4= 10°.
    trovato = false;
    
    resolution_q = deg2rad(15);
    
    q4 = joint_lim(4,1);  
    
    while q4 <= joint_lim(4,2) && ~trovato
        
        Q0 = cinematica_inversa_4gdl(traiettoria(1,1:2),traiettoria(1,3),link, q4)';
        if ~isempty(Q0)
            trovato = true;
            Q=Q0;
        end
        q4 = q4 + resolution_q;
    end
    
    XY_IK1=[];
    XY_IK2=[];
    XY_IK3=[];
    XY_IK4=[];
    XY_err=[];
    joints=[];
    
    % Calcolo il jacobiano simbolico qui fuori dal ciclo for cosi non devo
    % calcolarlo per ogni i ma lo faccio una sola volta e basta
    % Ja = calcolo_jacobiano_simbolico();
    % Ja=[Ja(1:2,:);Ja(6,:)];
    
    for i=1 : size(traiettoria,1)
        
        % Inversione cinematica
        Q_dot = inv_cin_psd(Q,traiettoria(i,1:3)',traiettoriaDot(i,1:3)', link, joint_lim);
        Q = Q + Q_dot*dt;
        
        joints = cat(1, joints, Q');
        [xy_ik1, xy_ik2, xy_ik3, xy_ik4] = kin_man_rid_progetto(Q, link);

        XY_IK1 = cat(1, XY_IK1, xy_ik1');
        XY_IK2 = cat(1, XY_IK2, xy_ik2');
        XY_IK3 = cat(1, XY_IK3, xy_ik3');
        XY_IK4 = cat(1, XY_IK4, xy_ik4');

        XY_err = cat(1, XY_err, xy_ik4(1:3)'- traiettoria(i,1:3));
    end
    
    figure;
    for i = 1 : 4
        subplot(4,1,i);
        plot(t,rad2deg(joints(:,i)),'-b','Linewidth',4)
        title(strcat("Angoli del giunto ", string(i)));
    end

    figure;
    posa = {'ascisse'; 'ordinata'; 'orientamento'};
    for i = 1 : 3
        subplot(3,1,i);
        plot(t,XY_err(:,i),'-b','Linewidth',4);
        title(strcat("Errore posa ", posa(i)));
    end
    
    prompt = "Salvare il video? [S][N] ";
    MAKE_VIDEO = lower(strip(input(prompt, 's')));
    
    if MAKE_VIDEO == "s"
        motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
        open(motion);
    end


    figure;
    for i = 1 : 20 : size(XY_IK1,1)

        plot(traiettoria(:,1), traiettoria(:,2),'-k','Linewidth',4)
        hold on
        plot([0 XY_IK1(i,1)],[0 XY_IK1(i,2)],'-r','Linewidth',4)
        plot([XY_IK1(i,1) XY_IK2(i,1)],[XY_IK1(i,2) XY_IK2(i,2)],'-b','Linewidth',4)
        plot([XY_IK2(i,1) XY_IK3(i,1)],[XY_IK2(i,2) XY_IK3(i,2)],'-g','Linewidth',4)
        plot([XY_IK3(i,1) XY_IK4(i,1)],[XY_IK3(i,2) XY_IK4(i,2)],'-y','Linewidth',4)
        
        axis equal
        xlim([-5 35])
        ylim([-10 27])




        if MAKE_VIDEO == "s"
            F = getframe(gcf);
            writeVideo(motion,F);
        end
        pause(0.5)
        hold off

    end



    if MAKE_VIDEO == "s"
        close(motion);
    end


end