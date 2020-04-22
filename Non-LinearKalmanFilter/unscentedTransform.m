function [x, P] = unscentedTransform(f, R, SP, W)
    n = size(SP,1);
    x = zeros(n,1);
    for i=1:numel(W)
        x = x + f(SP(:,i)) * W(i);
    end   
    P = R; %zeros(n,n);
    for i=1:numel(W)
        P = P + (f(SP(:,i))-x)*(f(SP(:,i))-x).' * W(i);
    end
end