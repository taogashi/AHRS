function y=INS_update(x,param)
% x: 7*1 state vector
% param: gyro rate w, time interval dt

y=zeros(size(x));
y(1:4) = QuatUpdate(x(1:4)',param(1:3),param(4))';
y(5:7) = x(5:7);

end
