function water_rocket_simulation()
    % Constants
    R = 8.3145; % J/(mol*kg)
    M = 0.028949; % kg/mol
    G = 6.67259e-11; % Gravitational constant (m^3/kg/s^2)
    m_Earth = 5.972e24; % kg
    radius_Earth = 6.371e6; % m
    rho_W = 997; % kg/m^3
    p_A0 = 1.01325e5; % Pa (at sea level)
    T = 288.15; % K (ISA standard temperature)

    % Inputs
    V_T = 1 / 1000; % Tank volume (m^3)
    m_W_s = 0.3; % Initial water mass (kg)
    m_Struktur = 0.15; % Structural mass (kg)
    m_Nutz = 0; % Payload mass (kg)
    p_P = 6 * 1e5; % Pump pressure (Pa)
    A_R = 0.00785; % Rocket cross-sectional area (m^2)
    d_D = 4 / 1000; % Nozzle diameter (m)
    c_w_R = 0.2; % Drag coefficient
    h_start = 0; % Starting height (m)
    dt = 0.01; % Time step (s)
    max_t = 20; % Maximum simulation time (s)

    % Derived values
    p_0 = p_A(h_start) + p_P; % Internal air pressure (Pa)
    V_W_s = m_W_s / rho_W; % Initial water volume (m^3)
    m_L_s = (V_T - V_W_s) * (p_0 * M / (R * T)); % Initial air mass (kg)
    A_D = pi * (d_D / 2)^2; % Nozzle area (m^2)
    m_R_s = m_Struktur + m_Nutz + m_L_s + m_W_s; % Initial total rocket mass (kg)

    % Initialize state variables
    h_R = h_start; % Rocket height (m)
    v_R = 0; % Rocket velocity (m/s)
    m_R = m_R_s; % Rocket mass (kg)
    V_W = V_W_s; % Water volume (m^3)
    m_L = m_L_s; % Air mass (kg)
    t = 0; % Time (s)

    % Storage for results
    time = [];
    heights = [];
    velocities = [];
    accelerations = [];

    while t < max_t && h_R >= 0
        rho_L = m_L / (V_T - V_W);
        p_L = rho_L * R * T / M;

        if p_L < p_A(h_R)
            p_L = p_A(h_R);
            rho_L = p_L * M / (R * T);
            m_L = rho_L * V_T;
        end

        if V_W > 0
            v_str = sqrt(2 * (p_L - p_A(h_R)) / rho_W);
            dV_W = v_str * A_D * dt;
            F_R_str = dV_W * rho_W * v_str;
            dv_R_str = F_R_str / m_R;
            dm_L = 0;
        else
            v_str = sqrt(2 * (p_L - p_A(h_R)) / rho_L);
            dm_L = v_str * A_D * dt * rho_L;
            F_R_str = dm_L * v_str;
            dv_R_str = F_R_str / m_R;
            dV_W = 0;
        end

        dv_R = dv_R_str + g(h_R) * dt;
        v_R = v_R + dv_R;
        h_R = h_R + v_R * dt;

        % Update rocket properties
        V_W = V_W - dV_W;
        if V_W < 0
            dV_W = V_W + dV_W;
            V_W = 0;
        end
        m_R = m_R - dV_W * rho_W - dm_L;
        m_L = m_L - dm_L;

        % Store results
        time(end+1) = t;
        heights(end+1) = h_R;
        velocities(end+1) = v_R;
        accelerations(end+1) = dv_R / dt;

        % Increment time
        t = t + dt;
    end

    % Plot results
    figure;
    subplot(3,1,1);
    plot(time, heights);
    title('Height vs Time');
    xlabel('Time (s)');
    ylabel('Height (m)');

    subplot(3,1,2);
    plot(time, velocities);
    title('Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');

    subplot(3,1,3);
    plot(time, accelerations);
    title('Acceleration vs Time');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');

    % Nested functions for environmental properties
    function p = p_A(h)
        p = p_A0 * (1 - 0.0065 * h / T);
    end

    function g_val = g(h)
        g_val = -(G * m_Earth) / (h + radius_Earth)^2;
    end
end
