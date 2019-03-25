function [XD, PHI] = calcola_tutti_punti_traiettoria()

    [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();

    dt = 0.5;
    
    t = (0 : dt : tempo.finale2)';
    XD = zeros(length(t), 2);
    XDD = zeros(length(t), 2);
    PHI = zeros(length(t), 1);
    PHID = zeros(length(t),1);
    for i = 1 : length(t)
        [XD(i,:),XDD(i,:),PHI(i),PHID(i)] = ...
            planner_TOAD(punto, tempo, phi, t(i), circonferenza1, circonferenza2);
    end
    
end