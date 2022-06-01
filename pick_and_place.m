clear
clc
%rosinit
l = [14.5, 10.7, 10.7, 9]; % Longitudes eslabones
n = 20;
%TC1 = ctraj(home,der,20);
%TC2 = ctraj(der,centro1,20);
%TC3 = ctraj(centro1,izq,20);
%TC4 = ctraj(izq,centro2,20);

%%

motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
motorCommandMsg = rosmessage(motorSvcClient); %Creación de mensaje
% Definicion del robot RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];

%%
ws = [-50 50]

home = transl(0,0,44.9)

cenI = transl(15,0,12)*troty(pi) %MTH Derecha intermedia
cenF = transl(15,0,7.5)*troty(pi) %MTH Derecha final
derI = transl(0,15,12)*trotx(-pi)*trotz(pi/2) %MTH Derecha intermedia
derF = transl(0,15,3.8)*trotx(-pi)*trotz(pi/2) %MTH Derecha final
izqI = transl(0,-15,12)*trotx(-pi)*trotz(-pi/2) %MTH Izquierda intermedia
izqF = transl(0,-15,3.8)*trotx(-pi)*trotz(-pi/2) %MTH Izquierda final
n = 20;
%Trayectorias
TH_DI = ctraj(home,derI,n); %Trayectoria de centro a posición derecha intermedia
TDI_DF = ctraj(derI,derF,n); %Trayectoria de centro a posición derecha intermedia
TDF_CI = ctraj(derF,cenI,n); %Trayectoria de derecho fina a posición central intermedia
TCI_CF = ctraj(cenI,cenF,n); %Trayectoria de derecho fina a posición central intermedia
TCF_II = ctraj(cenF,izqI,n); %Trayectoria de derecho fina a posición central intermedia
TII_IF = ctraj(izqI,izqF,n); %Trayectoria de derecho fina a posición central intermedia
TIF_CI = ctraj(izqF,cenI,n); %Trayectoria de derecho fina a posición central intermedia
TCF_CI = ctraj(cenF,cenI,n);
TCI_H  = ctraj(cenF,home,n); %Trayectoria de derecho fina a posición central intermedia
time = 0.01;
% for i=1:n*8
%     if i <= n
%         qinv = cinematica_inversa_px1(TH_DI(:,:,i))%Home to der_i
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)
%     elseif i <= 2*n
%         qinv = cinematica_inversa_px1(TDI_DF(:,:,i-n))% der_i to der_f
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)
%     elseif i <= 3*n
%         qinv = cinematica_inversa_px1(TDF_CI(:,:,i-2*n))%der_f to center_i
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)
%     elseif i <= 4*n
%         qinv = cinematica_inversa_px1(TCI_CF(:,:,i-3*n))%center_i to center_f
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)
%     elseif i <= 5*n
%         qinv = cinematica_inversa_px1(TCF_II(:,:,i-4*n))%center_f to izq_i
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)    
%     elseif i <= 6*n
%         qinv = cinematica_inversa_px1(TII_IF(:,:,i-5*n))%izq_i to izq_f
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time) 
%     elseif i <= 7*n
%         qinv = cinematica_inversa_px1(TIF_CI(:,:,i-6*n))%izq_f to _cen_i
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         
%         pause(time)
%     elseif i <= 8*n
%         qinv = cinematica_inversa_px1(TCF_CI(:,:,i-7*n))%cen_f to _home
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         pause(time)
%     elseif i <= 9*n
%         qinv = cinematica_inversa_px1(TCI_H(:,:,i-8*n))%cen_f to _home
%         PhantomX.plot(qinv(1,:),'notiles','noname')
%         pause(time) 
%     end
% end


%%

%
for i=1:n*9
    if i <= n
        qinv = rad2deg(cinematica_inversa_px1(TH_DI(:,:,i)));%Home to der_i
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=5;
        motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits 50 para cerrar
        call(motorSvcClient,motorCommandMsg);
        pause(time)
    elseif i <= 2*n
        qinv = rad2deg(cinematica_inversa_px1(TDI_DF(:,:,i-n)));% der_i to der_f
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        
    elseif i <= 3*n
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=5;
        motorCommandMsg.Value=round(mapfun(50,-150,150,0,1023));%bits 50 para cerrar
        call(motorSvcClient,motorCommandMsg);
        pause(time)
        qinv = rad2deg(cinematica_inversa_px1(TDF_CI(:,:,i-2*n)));%der_f to center_i
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        pause(time)
    elseif i <= 4*n
        qinv = rad2deg(cinematica_inversa_px1(TCI_CF(:,:,i-3*n)));%center_i to center_f
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        
    elseif i <= 5*n
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=5;
        motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits 50 para cerrar
        call(motorSvcClient,motorCommandMsg);
        pause(time)
        qinv = rad2deg(cinematica_inversa_px1(TCF_II(:,:,i-4*n)));%center_f to izq_i
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        pause(time)    
    elseif i <= 6*n
        qinv = rad2deg(cinematica_inversa_px1(TII_IF(:,:,i-5*n)));%izq_i to izq_f
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        
    elseif i <= 7*n
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=5;
        motorCommandMsg.Value=round(mapfun(50,-150,150,0,1023));%bits 50 para cerrar
        call(motorSvcClient,motorCommandMsg);
        pause(time)
        qinv = rad2deg(cinematica_inversa_px1(TIF_CI(:,:,i-6*n)));%izq_f to _cen_i
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        pause(time)
    elseif i <= 8*n
        qinv = rad2deg(cinematica_inversa_px1(TCI_CF(:,:,i-7*n)));%cen_f to _home
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(qinv(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        
    elseif i <= 9*n
        motorCommandMsg.AddrName="Goal_Position";
        motorCommandMsg.Id=5;
        motorCommandMsg.Value=round(mapfun(0,-150,150,0,1023));%bits 50 para cerrar
        call(motorSvcClient,motorCommandMsg);
        pause(time)
        qinv = rad2deg(cinematica_inversa_px1(TCI_H(:,:,i-8*n)));%cen_f to _home
        PhantomX.plot(qinv(1,:),'notiles','noname')
        for j=1:length(qinv(1,:))
            motorCommandMsg.AddrName="Goal_Position";
            motorCommandMsg.Id=j;
            motorCommandMsg.Value=round(mapfun(q_deg(1,j),-150,150,0,1023));%bits
            call(motorSvcClient,motorCommandMsg);
            pause(0.05);
        end
        
        pause(time) 
    end
end

hold on
trplot(eye(4),'rgb','arrow','frame',num2str(1),'length',15)
axis([repmat(ws,1,2) 0 60])