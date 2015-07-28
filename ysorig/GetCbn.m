function C=GetCbn(bA)
C=[ cos(bA(2))*cos(bA(3)) -cos(bA(1))*sin(bA(3))+sin(bA(1))*sin(bA(2))*cos(bA(3))  sin(bA(1))*sin(bA(3))+cos(bA(1))*sin(bA(2))*cos(bA(3));
    cos(bA(2))*sin(bA(3))  cos(bA(1))*cos(bA(3))+sin(bA(1))*sin(bA(2))*sin(bA(3)) -sin(bA(1))*cos(bA(3))+cos(bA(1))*sin(bA(2))*sin(bA(3));
   -sin(bA(2))             sin(bA(1))*cos(bA(2))                                      cos(bA(1))*cos(bA(2))];
end