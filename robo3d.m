clear all
a1 = 1; a2 = 1;
e = ETS3.Rz("q1")*ETS3.Ry("q2")* ...
ETS3.Tz(a1)*ETS3.Ry("q3")*ETS3.Tz(a2)* ...
ETS3.Rz("q4")*ETS3.Ry("q5")*ETS3.Rz("q6");
e.njoints
e.structure

%%
e.teach

%%
clear all;
a1 = 1; a2 = 1;

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform(se3([a1 0 0],"trvec"));
link3 = rigidBody("link3");
link3.Joint.setFixedTransform(se3([a2 0 0],"trvec"));

myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);

myRobot.showdetails

myRobot.show(deg2rad([30 40]));
view(0,90) % show only XY (2D) plane
%%
q = [linspace(0,pi,100)' linspace(0,-2*pi,100)'];
whos q


r = rateControl(10);
for i = 1:size(q,1)
    myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

%%
link2 = myRobot.Bodies{2}
link2 = myRobot.getBody("link2");

parentLink = link2.Parent
childLinks = link2.Children
link2.Joint.Type
link2.Joint.PositionLimits
myRobot.getTransform(deg2rad([0 30]),"link2","link1")

%% 3D corpo rigido Convertendo ets pra rbt
clear all

a1 = 1; a2 = 1;
robot6 = ets2rbt(ETS3.Rz("q1")*ETS3.Ry("q2")* ...
ETS3.Tz(a1)*ETS3.Ry("q3")*ETS3.Tz(a2)* ...
ETS3.Rz("q4")*ETS3.Ry("q5")*ETS3.Rz("q6"));
robot6.homeConfiguration
robot6.showdetails
robot6.show(deg2rad([30 40 10 20 30 90]));

%% Exercicio
clear all

L1 = 1; L2 = 1; L3 = 1;L4 =1;
%robot7ets = ETS3.Tz(L1)*ETS3.Rz("q1")*ETS3.Ty(L2)*ETS3.Rx("q2")*ETS3.Tx(-L3)*ETS3.Ty(L4)*ETS3.Ry("q3"); % Sem priorizar rotação em Z
robot7ets = ETS3.Tz(L1)*ETS3.Rz("q1")*ETS3.Ty(L2)*...
ETS3.Rx(deg2rad(-90))*ETS3.Rz("q2")*ETS3.Tx(-L3)*...
ETS3.Tz(L4)*ETS3.Rz("q3")*ETS3.Tx(0.2);

robot7ets.teach
%% Terminando exercicio
robot7 = ets2rbt(robot7ets);
robot7.homeConfiguration
robot7.showdetails
robot7.show(deg2rad([30 30 30]));
%%
q = [linspace(0,pi,100)' linspace(0,-2*pi,100)'linspace(0,-2*pi,100)'];
whos q


r = rateControl(10);
for i = 1:size(q,1)
    robot7.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end
