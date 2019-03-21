clc; clear; close all;
[punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();

xd = struct;
phid = struct;
t = 0 : 0.1 : 20;
for i = 1 : length(t)
    [xd(i).traiettoria,xd(i).derivata,phid(i).orientamento,phid(i).derivata] = ...
        planner_TOAD(punto, tempo, phi, t(i), circonferenza1, circonferenza2);
end

%Traiettoria di esempio nello spazio operativo
p=[ 30 20;
    xd(44).traiettoria(1) xd(44).traiettoria(2);
    xd(69).traiettoria(1) xd(69).traiettoria(2);
    20 20;
    xd(131).traiettoria(1) xd(131).traiettoria(2);
    xd(151).traiettoria(1) xd(151).traiettoria(2)];

theta=[0;
    phid(44).orientamento;
    phid(69).orientamento;
    pi/6;
    phid(131).orientamento;
    phid(151).orientamento];

link = ottimizza_link(p, theta);