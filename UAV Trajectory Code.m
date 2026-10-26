
clear; clc; close all;

t_total = 50;
dt = 0.1;
tspan = 0:dt:t_total;

y0 = [0 0 0 0.001 0.001];
tau_ = 0.1;

t_all = [];
y_all = [];

phase_ = 0;

for j = 1:length(tspan)-1

    if phase_ == 0
        if y0(1) < 30
            V_d = 10;
            a_d = 0;
        else
            phase_ = 1;
        end
    end

    if phase_ == 1
        if y0(2) < 10
            V_d = 10;
            a_d = 10;
        else
            phase_ = 2;
        end
    end

    if phase_ == 2
        V_d = 10;
        a_d = -10;
    end

    t_current = [tspan(j), tspan(j+1)];

    [t, y] = ode45(@(t,y) UAV_dynamics(t,y,V_d,a_d,tau_), ...
                   t_current, y0);

    y0 = y(end,:);

    y_all = [y_all; y(end,:)];%adds the last row of y below the existing rows of y_all, storing the UAV states at each iteration.
    t_all = [t_all; t(end)];%adds the last time value of t as a new row below the existing values of t_all, storing the simulation time at each iteration, although t is column vector so i can also write it as t(end,:) also

end

plot(y_all(:,1), y_all(:,2));

xlabel('x-position');
ylabel('y-position');
title('UAV Trajectory');

grid on;
axis equal;

function dydt = UAV_dynamics(t,y,V_d,a_d,tau_)

    gamma = y(3);
    V = y(4);
    a = y(5);

    dydt(1) = V*cos(gamma);
    dydt(2) = V*sin(gamma);
    dydt(3) = a/V;
    dydt(4) = (V_d - V)/tau_;
    dydt(5) = (a_d - a)/tau_;

    dydt = dydt.';

end
