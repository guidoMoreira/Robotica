clear all
link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link1.Joint.setFixedTransform([0 -pi/2 0.6718 0],"dh");
link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","fixed");
link2.Joint.setFixedTransform([0 0 0.05 0],"dh");
link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([0.432 0 0 0],"dh");
link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([0 0 -0.05 0],"dh");
link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([0.432 0 0 0],"dh");
link6 = rigidBody("link6");
link6.Joint = rigidBodyJoint("joint6","fixed");
link6.Joint.setFixedTransform([0 -pi/2 0 -pi/2],"dh");
link7 = rigidBody("link7");
link7.Joint = rigidBodyJoint("joint7","revolute");
link7.Joint.setFixedTransform([0.0 0 0.1 0],"dh");
link8 = rigidBody("link8");
link8.Joint = rigidBodyJoint("joint8","revolute");
link8.Joint.setFixedTransform([0.0 pi/2 0.1 0],"dh");
link9 = rigidBody("link9");
link9.Joint = rigidBodyJoint("joint9","fixed");
link9.Joint.setFixedTransform([0.0 0 0 pi/2],"dh");
link10 = rigidBody("link10");
link10.Joint = rigidBodyJoint("joint10","fixed");
link10.Joint.setFixedTransform([0.1 0 0 0],"dh");

%%
myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.addBody(link6,link5.Name);
myRobot.addBody(link7,link6.Name);
myRobot.addBody(link8,link7.Name);
myRobot.addBody(link9,link8.Name);
myRobot.addBody(link10,link9.Name);


% myRobot.showdetails
% myRobot.show(deg2rad([30 40 30]));
myRobot.show();

%% Exercicio slide 15
clear all
a = 1; alpha = 0; d = 0; theta = 0;
link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","fixed");
link1.Joint.setFixedTransform([0 0 10 0],"dh"); % a alpha d teta
link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","fixed");
link2.Joint.setFixedTransform([0 pi/2 0 0],"dh");
link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([50 0 0 0],"dh");
link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([40 0 0 0],"dh");
link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([30 0 0 0],"dh");
myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.showdetails
myRobot.show([0.5 1 1.5])
%%
T = myRobot.getTransform([0.5 1 1.5],"link5")

%%
while(1)
q = [linspace(0,pi/2,100)' linspace(0,pi/2,100)' linspace(0,pi/2,100)'];
whos q

r = rateControl(10);
for i = 1:size(q,1)
    myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end
end
%% Outro exercicio
clear all
L1 = 10; L2  = 10; L3 = 10; L4 = 10;

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","fixed");
link1.Joint.setFixedTransform([0 0 L1 0],"dh"); % a alpha d teta
link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([0 pi/2 0 0],"dh");
link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([0 0 L2 0],"dh");
link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","fixed");
link4.Joint.setFixedTransform([L3 0 0 0],"dh");
link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([0 0 L4 0],"dh");
link6 = rigidBody("link6");
link6.Joint = rigidBodyJoint("joint6","fixed"); % só pra visualizar
link6.Joint.setFixedTransform([0.2 0 0 0],"dh");


myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.addBody(link6,link5.Name);
myRobot.showdetails
myRobot.show()
%% 0 a 180
while(1)
q = [linspace(0,pi,100)' linspace(0,pi,100)' linspace(0,pi,100)'];
whos q

r = rateControl(10);
for i = 1:size(q,1)
    myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end
end