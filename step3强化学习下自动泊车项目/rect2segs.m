function [segsx,segsy] = rect2segs(rect)
for i = size(rect,1):-1:1
    x     = rect(i,1)';
    y     = rect(i,2)';
    w     = rect(i,3)';
    l     = rect(i,4)';
    theta = rect(i,5)';

    c = cos(theta);
    s = sin(theta);
    R = [c -s;s c];
    d = [-1 -1 1 1 -1;-1 1 1 -1 -1].*[w;l]./2;
    Z = R*d + [x;y];

    for j = 4:-1:1
        idx = (i-1)*4 + j;
        segsx(idx,:) = Z(1,j:(j+1));
        segsy(idx,:) = Z(2,j:(j+1));
    end
end