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

function h = h(xh, xv, yh, yv)
    h = sum(yh, 'all') + sum(yv, 'all');
end