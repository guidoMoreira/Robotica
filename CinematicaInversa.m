%% Cálculo Manual cinemática inversa
close all
L1=1
L2=1
r = rateControl(10);
q = [linspace(-0.5,0.5,100)' linspace(-1,-1,100)'];

for i = 1:size(q,1)
    clf
    %x=1.2*sin(q(i,1))
    %y=1.2*cos(q(i,2))
    x=q(i,1)
    y=q(i,2)

    %Equações obtidas algebricamente
    theta2rad = acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));
    theta1rad = atan2(y,x)-atan2(L2*sin(theta2rad),(L1+L2*cos(theta2rad)));
    
    %cinemática direta
    e1 = ETS2.Rz("q1")*ETS2.Tx(L1)
    e2 = ETS2.Rz("q1")*ETS2.Tx(L1)*ETS2.Rz("q2")*ETS2.Tx(L2);
    t1=e1.fkine([theta1rad])
    t2=e2.fkine([theta1rad theta2rad])

    line([0 t1(1,3) t2(1,3)],[0 t1(2,3) t2(2,3)])
    axis([-1 1 -2 0])
    hold off
	%myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
 end
%% Resolução automática das posições
clear all
syms a1 a2 real
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2);
syms q1 q2 real
TE = e.fkine([q1 q2])

transl = TE(1:2,3)';
syms x y real
e1 = x == transl(1)
e2 = y == transl(2)

[s1,s2] = solve([e1 e2],[q1 q2]);

length(s2)
subs(s2(2),[a1 a2],[1 1])

xfk = eval(subs(transl(1), [a1 a2 q1 q2], ...
[1 1 deg2rad(30) deg2rad(40)]))

yfk = eval(subs(transl(2), [a1 a2 q1 q2], ...
[1 1 deg2rad(30) deg2rad(40)]))

q1r = eval(subs(s1(2),[a1 a2 x y],[1 1 xfk yfk]));
q1 = rad2deg(q1r)

q2r = eval(subs(s2(2),[a1 a2 x y],[1 1 xfk yfk]));
q2 = rad2deg(q2r)

%% Resolvendo por otimização do erro
clear all
e = ETS2.Rz("q1")*ETS2.Tx(1)*ETS2.Rz("q2")*ETS2.Tx(1);
pstar = [0.6 0.7]; % Desired position
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])
printtform2d(e.fkine(q),unit="deg")
% exemplo anterior
pstar = [1.2080 1.4397];
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])

%% Exercicio Não FUNCIONA
clear all
L2 = 0.5; L3 = 0.4; L4 = 0.3;
e = ETS2.Rz("q1")*ETS2.Tx(L2)*ETS2.Rz("q2")*ETS2.Tx(L3)*ETS2.Rz("q3")*ETS2.Tx(L4);
syms q1 q2 q3 real
TE = e.fkine([q1 q2 q3]) % Pegar dados cinemática direta

transl = TE(1:2,3)';
syms x y z real
e1 = x == transl(1)
e2 = y == transl(2)
e3 = z == transl(3)

[s1,s2,s3] = solve([e1 e2 e3],[q1 q2 q3]);% resolver para equação 

%% Método numérico
clear all
L2 = 0.5; L3 = 0.4; L4 = 0.3;
e = ETS2.Rz("q1")*ETS2.Tx(L2)*ETS2.Rz("q2")*ETS2.Tx(L3)*ETS2.Rz("q3")*ETS2.Tx(L4);

pstar = [0.85 0.42]; % Desired position
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0 0])
printtform2d(e.fkine(q),unit="deg")
e.plot(q)
%% exe pt 2
clear all
L2 = 50; L3 = 40; L4 = 30;
e = ETS2.Rz("q1")*ETS2.Tx(L2)*ETS2.Rz("q2")*ETS2.Tx(L3)*ETS2.Rz("q3")*ETS2.Tx(L4);

%pstar = [linspace(70,70,100)' linspace(20,70,100)']

%Circulo com centro em (70, 70) e raio 20
x = 70 +20*cos(linspace(0,2*pi,100))
y = 70 +20*sin(linspace(0,2*pi,100))

pstar = [x' y'];

r = rateControl(10);
q = [pi -pi pi]
for i = 1:size(pstar,1)
    q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar(i,:)),q);
    e.plot(q,'workspace',[0 100 0 100 0 1]); % Plot pra 2d show pra 3d
    r.waitfor;
end

