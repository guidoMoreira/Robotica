clear all

%%

a1 = 1
e = ETS2.Rz("q1")*ETS2.Tx(a1)
e.fkine(pi/6) % Foward Kinematic

%% Mesma coisa mas fazendo pela matriz
se2(pi/6,"theta")*se2(eye(2),[a1 0])

%% Mostrar robo
e.teach

%
%% Robo com mais seguimentos
clear all
a1 = 1; a2 = 1;
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2)
T = e.fkine(deg2rad([30 40])); %Cinemática direta com 30 e 40
printtform2d(T,unit="deg")
%%
e.teach
%%
e.njoints
e.structure
e.plot(deg2rad([30 40]));

%% Bugado
clear all
e = ETS2.Rz("q1")*ETS2.Tx("q2",qlim=[1 2])
e.structure
e.teach
%% rotações z e y + translação + rotação y e z + translação + rotação z y z
clear all
a1 = 1; a2 = 1;

e = ETS3.Rz("q1")*ETS3.Ry("q2")*...
ETS3.Tz(a1)*ETS3.Ry("q3")*ETS3.Tz(a2)*...
ETS3.Rz("q4")*ETS3.Ry("q5")*ETS3.Rz("q6");
e.njoints
e.structure
e.teach
