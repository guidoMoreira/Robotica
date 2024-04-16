clear all

TA = trvec2tform([1 2])*tformr2d(30,"deg") % Translação depois rotação
TC = tformr2d(30,"deg")* trvec2tform([1 2]) % rotação depois translação

%Exe1
TB =  trvec2tform([2 1])
TAB = TA*TB
TBA = TB*TA


p = [3;2];


% Pb = inversa de TB * p0
PB=inv(TB)*[3 2 1]'

PA = inv(TA)*TB * PB

plottform2d(TA,frame="A",color="b")
plottform2d(TB,frame="B",color="r")
%plottform2d(TC,frame="C",color="y")
plottform2d(TAB,frame="TAB",color="c")
plottform2d(TBA,frame="TBA",color="g")
plotpoint(p',"ko",label="P");

grid

axis([0 5 0 5])% grafico de 0 a 5
%% Exe 2
clear all

T0 = trvec2tform([0 0]);
plottform2d(T0,frame="0",color="r");
TX = trvec2tform([2 3]);
plottform2d(TX,frame="X",color="b");

TR = tformr2d(2);
plottform2d(TR,frame="TR",color="c");
plottform2d(TR*TX,frame="RX",color="b");
plottform2d(TX*TR,frame="XR",color="b");

C = [3 2];
plotpoint(C,"ko",label="C");

TC = trvec2tform(C)*TR*trvec2tform(-C)
plottform2d(TC*TX,frame="XC",color="r");

axis([-5 4 -1 5])
grid
%% Exe 3

alpha = 30
beta = 0
TR=tformr2d(alpha,"deg")*...
    trvec2tform([0.6 0])*...
    tformr2d(beta,"deg")*...
    trvec2tform([0.7 0])