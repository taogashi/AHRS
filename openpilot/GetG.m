function y = GetG(x, dt)
y = 0.5*dt*[-x(2) -x(3) -x(4) 0 0 0;
            x(1) -x(4) x(3) 0 0 0;
            x(4) x(1) -x(2) 0 0 0;
            -x(3) x(2) x(1) 0 0 0;
            0 0 0 1 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1];
end