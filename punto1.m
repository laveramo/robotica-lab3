%% Modelo cinemática inversa Phantom X
% Se encuentra en la función llamada: cinematica_inversa_px.m
% Es una función que recibe una MTH de la postura de la herramienta.
pose = [1 0 1 11; 0 -1 1 11; 1 0 1 25; 0 0 0 1]
q = cinematica_inversa_px(pose)
%% Comprobación del anterior modelo con el toolbox de Peter Corke
l = [14.5, 10.7, 10.7, 9]; % Longitudes eslabones
% Definicion del robot RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
q_toolbox = PhantomX.ikunc(pose)

%% Esbozo del espacio de trabajo del Phantom X