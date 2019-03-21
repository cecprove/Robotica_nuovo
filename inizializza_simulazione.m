function [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione()
punto = struct;
punto.iniziale = [30 20];
punto.finale = [20 20];

tempo = struct;
tempo.iniziale = 0;
tempo.finale1 = 10;
tempo.finale2 = 20;

phi.iniziale = 0;
phi.finale = pi/6;

circonferenza1 = struct;
circonferenza1.centro = [25 34.1]; %centro circonferenza
circonferenza1.raggio = 15;

circonferenza2 = struct;
circonferenza2.centro = [25 11];
circonferenza2.raggio = 10;
end