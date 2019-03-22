clc; clear; close all;
[punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();

XD = struct;
PHI = struct;
xd = [];
yd = [];
phid = [];

prompt = 'Come vuoi pianificare la traiettoria? [F]or - [S]imulink: ';
str = input(prompt,'s');

if strip(lower(str)) == "s"
    puntoS = timeseries([punto.iniziale; punto.finale]);
    tempoS = timeseries([tempo.iniziale; tempo.finale1; tempo.finale2]);
    phiS = timeseries([phi.iniziale, phi.finale]);
    centri_circonferenzeS = timeseries([circonferenza1.centro; circonferenza2.centro]);
    raggi_circonferenzeS = timeseries([circonferenza1.raggio; circonferenza2.raggio]);
    sim("TOAD_simulation");
    xd = XD(1,:);
    yd = XD(2,:);
    phid = PHI;
else
    t = 0 : 0.05 : 20;
    for i = 1 : length(t)
        [XD(i).traiettoria,XD(i).derivata,PHI(i).orientamento,PHI(i).derivata] = ...
            planner_TOAD(punto, tempo, phi, t(i), circonferenza1, circonferenza2);
    end
    xd = XD(:).traiettoria(1);
    yd = XD(:).traiettoria(2);
    phid = PHI(:).orientamento;
end

[p, theta] = calcola_punti_traiettoria(xd, yd, phid);

link = ottimizza_link(p, theta);
