function y=GetA(gyrRate,dT)
W=[0 -gyrRate(1) -gyrRate(2) -gyrRate(3);
    gyrRate(1) 0 gyrRate(3) -gyrRate(2);
    gyrRate(2) -gyrRate(3) 0 gyrRate(1);
    gyrRate(3) gyrRate(2) -gyrRate(1) 0];
Sig=W*dT;
y=eye(size(Sig))+Sig/2;
end