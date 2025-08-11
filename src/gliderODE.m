function dydt = gliderODE(~, y, params) % Passing in state vector + extra params
     
    % State vector components; note x and h are unused

    vx = y(1);
    vy = y(2);
    % x = y(3); 
    % h = y(4);

    Clift = params(1);
    Cdrag = params(2);
    g = params(3);
    m = params(4);
    rho = params(5);
    A = params(6);

    % Velocity magnitude

    vMag = max(hypot(vx, vy), 1e-6); % In the case vx and vy = 0, we prevent division by 0

    % Dynamic Pressure, Lift and Drag

    q = (1/2)*rho*((vMag)^2);

    L = Clift*q*A;
    D = Cdrag*q*A;

    % Accelerations 

    ax = -D*vx/(vMag*m);
    ay = (L/m)-(D*vy)/(m*vMag)-g;

    % Derivatives vector

    dydt = [ax; ay; vx; vy];

end

