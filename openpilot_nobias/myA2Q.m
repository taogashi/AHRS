function y=myA2Q(Angle)
sinFi=sin(Angle(1)/2);cosFi=cos(Angle(1)/2);
sinTh=sin(Angle(2)/2);cosTh=cos(Angle(2)/2);
sinPe=sin(Angle(3)/2);cosPe=cos(Angle(3)/2);

y=zeros(1,4);
y(1)=cosFi*cosTh*cosPe+sinFi*sinTh*sinPe;
y(2)=sinFi*cosTh*cosPe-cosFi*sinTh*sinPe;
y(3)=cosFi*sinTh*cosPe+sinFi*cosTh*sinPe;
y(4)=cosFi*cosTh*sinPe+sinFi*sinTh*cosPe;

end