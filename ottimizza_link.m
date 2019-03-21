function link = ottimizza_link(p, theta)

    [joint_lim, link_lim] = inizializza_limiti();
    
    %Range di variazione dei link nel metodo di ottimizzazione
    passo_crescita=1;

    %Vettori vuoti
    link_scelti = [];
    somma_link = [];
    differenza_link = [];
    link_scartati = [];

    
    iterazioni_tot = 1;
    for k = 1 : size(link_lim, 1) 
        iterazioni_k = ((link_lim(k,2) - link_lim(k,1))/passo_crescita) + 1;
        iterazioni_tot = iterazioni_tot * iterazioni_k;
    end

    resolution_q=deg2rad(15);
    %iterazioni_tot = iterazioni_tot * ((joint_lim(4,2)-joint_lim(4,1))/resolution_q);
    
    conta_iterazioni = 0;
    barra = waitbar(0,'please wait', 'Name', 'Barra di caricamento');

    for a1 = link_lim(1,1) : passo_crescita : link_lim(1,2)
        for a2 = link_lim(2,1) : passo_crescita : link_lim(2,2)
            for a3 = link_lim(3,1) : passo_crescita : link_lim(3,2)
                for a4 = link_lim(4,1) : passo_crescita : link_lim(4,2)
                    
                    progress = conta_iterazioni/iterazioni_tot;
                    waitbar(progress, barra, sprintf("Running %.1f%%", progress * 100));
                    conta_iterazioni = conta_iterazioni + 1;
                        
                    trovato = false;
                    q4 = joint_lim(4,1);
                    
                    while q4 <= joint_lim(4,2) && ~trovato
                    %for q4 = joint_lim(4,1) : resolution_q : joint_lim(4,2)

                        check = controlla_dimensioni_link(p, theta, q4, [a1,a2,a3,a4], joint_lim);


                        %Ciclo if
                        %& se check è vero inserisco in link a1,a2,a3 e se voglio
                        %conservare anche quelli precedenti (dove c'è il check vero)
                        %scrivo links=[links; a1...]
                        if check
                            link_scelti = cat(1, link_scelti, [a1, a2, a3, a4]);
                            %& devo poi realizzare quella funzione che mi va a
                            % minimizzare quelle funzioni di costo che ho scelto prima

                            %& prima funzione di costo è la somma dei link quindi creo
                            %una variabile contenente le somme di tutti i link
                            sum_link = a1+a2+a3+a4;
                            somma_link = cat(1, somma_link, sum_link);
                            %& mi permettono di trovare il massimo delle combinazioni
                            %dei tre link
                            max_1234=max([abs(a1-(sum_link)/4),abs(a2-(sum_link)/4),abs(a3-(sum_link)/4),abs(a4-(sum_link)/4)]);

                            differenza_link = cat(1, differenza_link, max_1234);
                            
                            trovato = true;
                        end
                        %& c'è _ .Qui metto tutte le combinazioni che ho trovato anche
                        %quelle che non vanno bene
                        q4 = q4 + resolution_q;
                        link_scartati = cat(1, link_scartati, [a1, a2, a3, a4]);
                    end
                end
            end
        end
    end
    
    close(barra);
    
    %Vengono applicate delle funzioni di costo
    % tipo deviazione stadard
    somma_link = somma_link - mean(somma_link);
    differenza_link = differenza_link - mean(differenza_link);
    %& normalizzo solitamnete divido tutti gli elementi del vettore per la
    %differenza tra il massimo e il minimo e cosi poi tutti gli elementi del
    %vettore varieranno tra 0 e 1.
    links_sum_norm = somma_link / abs((max(somma_link) - min(somma_link)));
    
    links_diff_norm = differenza_link/abs((max(differenza_link)-min(differenza_link)));

    cost_function= links_sum_norm + links_diff_norm;
    
    plot_ottimizza_link(somma_link, differenza_link, cost_function, links_sum_norm, links_diff_norm);
    
    [~, correct_ind] = min(cost_function);
    %& cosi salvo la combinazione corretta che meglio minimizza il funzionale
    %di costo
    link = link_scelti(correct_ind,:);
end