function [ qx ] = skew( q )   
    qx = [ 0   -q(3)  q(2);
          q(3)   0   -q(1);
         -q(2)  q(1)   0 ];
end

function [q] = invQuat(q)
q = [-1*q(1:3);q(4)];
end

function [c] = convertToMatrix(q)
    c = eye(3) - 2*q(4)*skew(q) + 2*(skew(q)*skew(q));
    % We have a left-hand-rule rotation matrix because of the current quaternion convention. 
end

function [normalized] = NormalizeV(aVector)
    normalized = aVector./norm(aVector);
end

function [omega] = Omega(w)
    omega = [ 0    w(3) -w(2) w(1);
             -w(3) 0     w(1) w(2);
              w(2) -w(1) 0    w(3);
             -w(1) -w(2) -w(3) 0 ];
end

function [result] = multiplyQuaternion(q, p)
    result = [( q(4)*p(1) + q(3)*p(2) - q(2)*p(3) + q(1)*p(4));
              (-q(3)*p(1) + q(4)*p(2) + q(1)*p(3) + q(2)*p(4));
              ( q(2)*p(1) - q(1)*p(2) + q(4)*p(3) + q(3)*p(4));
              (-q(1)*p(1) - q(2)*p(2) - q(3)*p(3) + q(4)*p(4))];
end


