function q = cinematica_inversa_px(pose)
    % q = vector de soluciones
    q = zeros(2,4);
    l = [14.5, 10.7, 10.7, 9]; % Longitudes eslabones
    if size(pose) == [4, 4]
        % q1 = tan(py/px)
        q1 = atan2(pose(2,4),pose(1,4));
        q(:,1) = q1;
        
        % q2 y q3
        Posw = pose(1:3,4) - l(4)*pose(1:3,3); 
        h = Posw(3) - l(1);
        r = sqrt(Posw(1)^2 + Posw(2)^2);

        % Codo abajo
        the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
        the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));

        q2d = -(pi/2-the2);
        q3d = the3;
        
        q(1,2) = q2d;
        q(1,3) = q3d;

        % Codo arriba
        % the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
        the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
        q2u = -(pi/2-the2);
        q3u = -the3;
        q(2,2) = q2u;
        q(2,3) = q3u;
        
        % q4
        Rp = (rotz(q1))'*pose(1:3,1:3);
        pitch = atan2(Rp(3,1),Rp(1,1));

        q4d = pitch - q2d - q3d;
        q4u = pitch - q2u - q3u;
        %disp(rad2deg([q4d q4u]))

        q(1,4) = q4d;
        q(2,4) = q4u;
        
        q = rad2deg(q);
        
    else
        disp("Error: Matrix pose dimensions wrong")
    end
    
end