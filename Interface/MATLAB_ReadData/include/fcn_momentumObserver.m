function [r,phat_out] = fcn_momentumObserver(M,dq,u,dt,Ki,r_prev,p_hat)

%%
% persistent r_prev
% if isempty(r_prev)
%     r_prev = zeros(size(M,1),1);
% end
p = M.*dq;
duk = u - r_prev;

%%
% persistent p_hat
% if isempty(p_hat)
%     p_hat = zeros(size(M,1),1);
% end
phat_out = p_hat + duk*dt;

%%
r = Ki.*(p_hat-p);
% r_prev = r;


end

