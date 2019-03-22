clc; clear; close all;
[punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();

xd = [];
yd = [];
phid = [];
t = [];

dt = 0.05;

prompt = 'Come vuoi pianificare la traiettoria? [F]or - [S]imulink: ';
str = input(prompt,'s');

if strip(lower(str)) == "s"
    puntoS = timeseries([punto.iniziale; punto.finale]);
    tempoS = timeseries([tempo.iniziale; tempo.finale1; tempo.finale2]);
    phiS = timeseries([phi.iniziale, phi.finale]);
    centri_circonferenzeS = timeseries([circonferenza1.centro; circonferenza2.centro]);
    raggi_circonferenzeS = timeseries([circonferenza1.raggio; circonferenza2.raggio]);
    
    simulazione = sim("TOAD_simulation", 'StartTime',num2str(tempo.iniziale),...
        'StopTime',num2str(tempo.finale2),...
        'FixedStep',num2str(dt));
    
    xd = simulazione.XD(:,1);
    yd = simulazione.XD(:,2);
    phid = simulazione.PHI;
    t = simulazione.tout;
else
    t = (0 : dt : tempo.finale2)';
    XD = zeros(length(t), 2);
    XDD = zeros(length(t), 2);
    PHI = zeros(length(t), 1);
    PHID = zeros(length(t),1);
    for i = 1 : length(t)
        [XD(i,:),XDD(i,:),PHI(i),PHID(i)] = ...
            planner_TOAD(punto, tempo, phi, t(i), circonferenza1, circonferenza2);
    end
    xd = XD(:,1);
    yd = XD(:,2);
    phid = PHI;
end

[p, theta] = calcola_punti_traiettoria(xd, yd, phid);

link = ottimizza_link(p, theta);
