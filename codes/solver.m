function [q_new, u]=solver(t, q, q_d, delta_Q, drp, init_params, j)
u = calc_u(t, q, q_d, delta_Q, drp, init_params, j);
dt = init_params.dt;
method = 'r';   % r: Runge-kutta, e: Euler
if method == 'e'
    dq = robot(q, u);
    q_new = q+dt*dq;
elseif method == 'r'
    k1=dt*robot(q, u);
    k2=dt*robot(q+k1/2, u);
    k3=dt*robot(q+k2/2, u);
    k4=dt*robot(q+k3, u);
    q_new=q+1/6*(k1+2*k2+2*k3+k4);
end