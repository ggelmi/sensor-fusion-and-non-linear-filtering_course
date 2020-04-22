function [x,y] = normpdf2(mu, sigma2, level, N)
    x = linspace(mu-level*sqrt(sigma2), mu+level*sqrt(sigma2), N);
    y = normpdf(x, mu, sqrt(sigma2));
end