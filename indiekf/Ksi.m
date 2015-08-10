function y = Ksi(q)
y = [q(4) * eye(3) + skew(q(1:3));
    -q(1:3)'];
end