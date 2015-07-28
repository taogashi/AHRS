function y=GetEstiOb(x,m0)
% x: 5x1 state vector
% m0: normalized geomagnetism
% r: radias of the driller

y=zeros(6,1);
Cbn = myQuat2Cbn(x(1:4));
y(1:3) = Cbn'*[0;0;-1];
y(4:6) = Cbn'*m0;

end