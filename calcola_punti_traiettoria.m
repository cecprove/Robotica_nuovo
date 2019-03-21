function [p, theta] = calcola_punti_traiettoria(xd, yd, phi)
    distanze_punti = [];
    for i = 1 : length(xd)
        dist_punto = sqrt(xd(i)^2 + yd(i)^2);
        distanze_punti = cat(1, distanze_punti, dist_punto);
    end
    
    [~, max_ind] = max(distanze_punti);
    [~, min_ind] = min(distanze_punti);
    %Traiettoria di esempio nello spazio operativo
    p=[ 30 20;
        xd(max_ind) yd(max_ind);
        20 20;
        xd(min_ind) yd(min_ind)];

    theta=[0;
        phi(max_ind);
        pi/6;
        phi(min_ind)];
end