function y=GetA(x, gyrRate,dT)
W=[0 x(5)-gyrRate(1) x(6)-gyrRate(2) x(7)-gyrRate(3) x(2) x(3) x(4);
    gyrRate(1)-x(5) 0 gyrRate(3)-x(7) x(6)-gyrRate(2) -x(1) x(4) -x(3);
    gyrRate(2)-x(6) x(7)-gyrRate(3) 0 gyrRate(1)-x(5) -x(4) -x(1) x(2);
    gyrRate(3)-x(7) gyrRate(2)-x(6) x(5)-gyrRate(1) 0 x(3) -x(2) -x(1);
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0];
Sig=W*dT;
y=eye(size(Sig))+Sig/2; %zero order integration
end