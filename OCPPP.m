% Created on May 24 2021
% 
% @author: Wei-Yu Chen
% Purpose: Optimal Coverage Path Planning Problem(OCPPP)
% Optimization problem:(total map is assumed to be nxn)
% L(x) = min d + h(x) + \norm (J^2)x \norm_1
% s.t.
% x_i - x_(i-1) \in {-1, 1, -n, n} (moves are restricted to up down left and right)
% x_i \in [0, n^2-1] (all trajectories can't leave the boundary of the map)
% d = len(x) <= n^2-1
% h(x) = ({O \cap x == \emptyset)?0:10000 (O is the set of obstacles within the map, punish trajectories that traverse obstacles)
% {0,1,2...n^2-1}\O \in x (the whole map must be covered except for obstacles)
% x_0 \in I (set of starting points)
% x_d-1 \in E (set of ending points)
% J is nx(n-1) matrix
% we have ourselves a convex optimization problem

NAIs = load("NAI.mat");
Ah = NAIs.NAIH;
Ah = cast(Ah, "double");
Av = NAIs.NAIV;
Av = cast(Av, "double");
a = size(Ah);
top_right = zeros(a(1), a(2));
bottom_left = zeros(a(1), a(2));
A = [Ah top_right; bottom_left Av];
xh = optimvar('xh',a(2),'Type','integer','LowerBound',0,'UpperBound',1);
xv = optimvar('xv',a(2),'Type','integer','LowerBound',0,'UpperBound',1);
yh = optimvar('yh',a(1),'Type','integer','LowerBound',0,'UpperBound',1);
yv = optimvar('yv',a(1),'Type','integer','LowerBound',0,'UpperBound',1);
obj = h(xh,xv,yh,yv);
prob = optimproblem('Objective', obj);
ineq_xh = Ah*xh<=yh;
ineq_xv = Av*xv<=yv;
eq_x = xh + xv == ones(a(2),1);
eq_y = yh + yv == ones(a(1),1);
prob.Constraints.ineq1 = ineq_xh;
prob.Constraints.ineq2 = ineq_xv;
prob.Constraints.eq_x = eq_x;
% prob.Constraints.eq_y = eq_y;
sol = solve(prob);
sol.xh
sol.xv;
sol.yh;
sol.yv;
writematrix(sol.xh, 'xh.csv')
% n = 10;
% O = [];
% for i=2*n:n^2-1
%     x = optimvar('x',i,'Type','integer','LowerBound',0,'UpperBound',n^2-1);
%     obj = i + h(x,O) + b(x);
%     prob = optimproblem('Objective', obj);
% %     show(prob)
%     x0.x = zeros(1,i);
%     [sol,fval,exitflag,output] = solve(prob,x0)
% end

% x = optimvar('x',99,'Type','integer','LowerBound',0,'UpperBound',n^2-1);
% obj = 99 + h(x,O) + b(x);
% prob = optimproblem('Objective', obj);
% x0.x = zeros(1,99);
% [sol,fval,exitflag,output] = solve(prob,x0);
% sol

function h = h(xh, xv, yh, yv)
    h = sum(yh, 'all') + sum(yv, 'all');
end

% function dd = b(x)
%     total = 0;
%     for i=3:length(x)
%         firstder = x(i-1) - x(i-2);
%         secder = x(i)-x(i-1) - firstder;
%         total = ((total + secder)^2)^0.5;
%     end
%     dd = total;
% end
