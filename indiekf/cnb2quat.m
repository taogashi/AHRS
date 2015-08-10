function y = cnb2quat(cnb)
T = trace(cnb);
q4 = sqrt(1+T)/2;
y = [(cnb(2,3) - cnb(3,2))/(4*q4);
    (cnb(3,1) - cnb(1,3))/(4*q4);
    (cnb(1,2) - cnb(2,1))/(4*q4);
    q4];
end