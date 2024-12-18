function h = Create3DAxis(TMat)
%TMat   : 4x4 Transformation matrix
vo = TMat(1:3,4);
v1 = [1; 0; 0; 1];
v2 = [0; 1; 0; 1];
v3 = [0; 0; 1; 1];
VT = TMat*v1; 
Vx = VT(1:3);
VT = TMat*v2; 
Vy = VT(1:3);
VT = TMat*v3; 
Vz = VT(1:3);
% light on
light('Position',[0 0 1],'Style','infinite');
mArrow3(vo', Vx','color','red','stemWidth',0.01,'facealpha',0.9);
mArrow3(vo', Vy','color','blue','stemWidth',0.01,'facealpha',0.9);
mArrow3(vo', Vz','color','green','stemWidth',0.01,'facealpha',0.9);
