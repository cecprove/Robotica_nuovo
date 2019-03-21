function [joint_lim, link_lim] = inizializza_limiti()
    %Limiti di giunto del manipolatore
    joint_lim=[deg2rad(-70) deg2rad(70);
        deg2rad(0) deg2rad(140);
        deg2rad(0) deg2rad(140);
        deg2rad(-90) deg2rad(90)];
    
    %Grandezza minima/massima dei link
    link_lim=[5 18; 
        5 18;
        5 18; 
        5 18];

end