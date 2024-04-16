%% Método numérico
clear all
L1 = 660; L2 = 432; L3 = 432; L4 = 200;
e = ETS3.Tz(L1)*ETS3.Rz("q1")*ETS3.Rx(pi/2)*ETS3.Rz("q2")*ETS3.Tx(L2)*ETS3.Rz("q3")*ETS3.Tx(L3)*ETS3.Rx("q4")*ETS3.Rz("q5")*ETS3.Rx("q6")*ETS3.Tx(L4);

pstar = [800 42 0]; % Desired position
q = fminsearch(@(q) norm(se3(e.fkine(q)).trvec-pstar),[0 0 0 0 0 0])
%printtform2d(e.fkine(q),unit="deg")
e.plot(q) %([pi/4 pi/4 pi/4 pi/4])
%% exe pt 2
clear all
L1 = 660; L2 = 432; L3 = 432; L4 = 200;
e = ETS3.Tz(L1)*ETS3.Rz("q1")*ETS3.Rx(pi/2)*ETS3.Rz("q2")*ETS3.Tx(L2)*ETS3.Rz("q3")*ETS3.Tx(L3)*ETS3.Rx("q4")*ETS3.Rz("q5")*ETS3.Rx("q6")*ETS3.Tx(L4);

%pstar = [linspace(70,70,100)' linspace(20,70,100)']

%Circulo com centro em (70, 70) e raio 20
x = linspace(700,700,100)
y = 200*cos(linspace(0,2*pi,100))
z = 600 +200*sin(linspace(0,2*pi,100))

pstar = [x' y' z'];

r = rateControl(100);
q = [0 0 0 0 0 0]


for i = 1:size(pstar,1)
    q = fminsearch(@(q) norm(se3(e.fkine(q)).trvec-pstar(i,:)),q);
    e.plot(q,'workspace',[-500 1000 -500 1000 -500 1200]); % Plot pra 2d show só pra 3d
    r.waitfor;
end

%% exe mas reto no fim
clear all
L1 = 660; L2 = 432; L3 = 432; L4 = 200;
e = ETS3.Tz(L1)*ETS3.Rz("q1")*ETS3.Rx(pi/2)*ETS3.Rz("q2")*ETS3.Tx(L2)*ETS3.Rz("q3")*ETS3.Tx(L3)*ETS3.Rx("q4")*ETS3.Rz("q5")*ETS3.Rx("q6")*ETS3.Tx(L4);

%pstar = [linspace(70,70,100)' linspace(20,70,100)']

%Circulo com centro em (70, 70) e raio 20
x = linspace(700,700,100)
y = 200*cos(linspace(0,2*pi,100))
z = 600 +200*sin(linspace(0,2*pi,100))

pstar = [linspace(700,700,100)'...
    200*cos(linspace(0,2*pi,100))'...
    600+200*sin(linspace(0,2*pi,100))'...
    linspace(0,0,100)'...
    linspace(0,0,100)'...
    linspace(0,0,100)'];

r = rateControl(100);
q = [0 0 0 0 0 0]


for i = 1:size(pstar,1)
    q = fminsearch(@(q) norm([se3(e.fkine(q)).trvec se3(e.fkine(q)).eul]-pstar(i,:)),q);
    e.plot(q,'workspace',[-500 1000 -500 1000 -500 1200]); % Plot pra 2d show só pra 3d
    r.waitfor;
end