function res = QuatProd(q,p)
if numel(p) == 3
    p = [0; p];
end

Q = [q(1), -q(2), -q(3), -q(4);
     q(2), q(1), -q(4), q(3);
     q(3), q(4), q(1), -q(2);
     q(4), -q(3), q(2), q(1)];
res = Q * p;
end