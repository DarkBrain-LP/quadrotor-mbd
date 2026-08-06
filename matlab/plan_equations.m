% % Paramètres physiques (valeurs arbitraires réalistes pour test)
% I_xx = 0.02;      % moment d'inertie en roll, kg·m² (valeur typique petit drone)
% I_yy = 0.02;      % moment d'inertie en roll, kg·m² (valeur typique petit drone)
% I_zz = 0.02;      % moment d'inertie en roll, kg·m² (valeur typique petit drone)
% d = 0.1;          % bras de levier, mètres
% k_m = 0.02;              % coefficient d'inertie moteur (ration couple/poussée)
% k_f = 1;              % coefficient d'inertie 
% 
% Fm1 = 2.5; Fm2 = 2.0; Fm3 = 2.0; Fm4 = 2.5;   % poussées moteur, Newtons (test : léger déséquilibre gauche/droite)
% 
% b_roll = 0;       % coefficient d'amortissement du couple de trainée aérodynamique de rotation
% b_pitch = 0;       % coefficient d'amortissement du couple de trainée aérodynamique de rotation
% b_yaw = 0;       % coefficient d'amortissement du couple de trainée aérodynamique de rotation
% 
% % Calcul du Torque_roll à partir des 4 poussées (équation qu'on a déjà vérifiée ensemble)
% Torque_roll = d * (Fm1 - Fm2 - Fm3 + Fm4);
% Torque_pitch = d * (Fm3 + Fm4 - Fm1 - Fm2);
% Torque_yaw = (k_m/k_f) * (Fm2 - Fm1 + Fm4 - Fm3);
% 
% % Fonction qui définit dx/dt pour ode45
% function dxdt = roll_dynamics(t, x, Torque_roll, I)
%     angle_roll = x(1);
%     omega_roll = x(2);
% 
%     Torque_net_roll = Torque_roll - b_roll * omega_roll
% 
%     d_angle_roll = omega_roll;              % définition de omega
%     d_omega_roll = Torque_net_roll / I;                     % <-- À TOI DE COMPLÉTER
% 
%     dxdt = [d_angle_roll; d_omega_roll];
% end
% 
% function dydt = pitch_dynamics(t, x, Torque_pitch, I)
%     w_pitch = x(2);
% 
%     Torque_net_pitch = Torque_pitch - b_pitch * w_pitch;
% 
%     d_angle_pitch = w_pitch ;              % définition de omega
%     d_w_pitch  = Torque_net_pitch / I;                     % <-- À TOI DE COMPLÉTER
% 
%     dydt = [d_angle_pitch ; d_w_pitch];
% end
% 
% 
% function dzdt = yaw_dynamics(t, x, Torque_yaw, I)
%     w_yaw = x(2);
% 
%     Torque_net_yaw = Torque_yaw - b_yaw * w_yaw;
% 
%     d_angle_yaw = w_yaw;              % définition de omega
%     d_w_yaw = Torque_net_yaw / I;                     % <-- À TOI DE COMPLÉTER
% 
%     dzdt = [d_angle_yaw; d_w_yaw];
% end
% 
% 
% % Conditions initiales et intégration
% x0 = [0; 0];              % angle=0, omega=0 au départ (drone stable)
% y0 = [0; 0];
% z0 = [0; 0];
% tspan = [0 2];             % simuler de t=0 à t=2 secondes
% 
% [t, x] = ode45(@(t,x) roll_dynamics(t, x, Torque_roll, I_xx), tspan, x0);
% [t, y] = ode45(@(t,y) pitch_dynamics(t, y, Torque_pitch, I_yy), tspan, y0);
% [t, z] = ode45(@(t,z) yaw_dynamics(t, z, Torque_yaw, I_zz), tspan, z0);
% 
% % Affichage
% plot(t, rad2deg(x(:,1))); xlabel('Temps (s)'); ylabel('Angle roll (deg)');1
% 
% % plot(t, x(:,2)); xlabel('Temps (s)'); ylabel('omega roll (rad/s)');1

%% Paramètres physiques (valeurs arbitraires réalistes pour test)
I_xx = 0.02;      % moment d'inertie en roll, kg·m²
I_yy = 0.02;      % moment d'inertie en pitch, kg·m²
I_zz = 0.02;      % moment d'inertie en yaw, kg·m²
d = 0.1;          % bras de levier, mètres
k_m = 0.02;       % coefficient de couple de réaction (valeur arbitraire, à mesurer plus tard)
k_f = 1.0;        % coefficient de poussée (valeur arbitraire, à mesurer plus tard)

Fm1 = 2.5; Fm2 = 4.0; Fm3 = 1.0; Fm4 = 6.5;   % poussées moteur, Newtons (déséquilibre gauche/droite)

b_roll = 0;    % coefficient d'amortissement aérodynamique — désactivé (hypothèse simplifiée documentée)
b_pitch = 0;
b_yaw = 0;

%% Calcul des Torques à partir des 4 poussées
Torque_roll  = d * (Fm1 - Fm2 - Fm3 + Fm4);
Torque_pitch = d * (Fm3 + Fm4 - Fm1 - Fm2);
Torque_yaw   = (k_m/k_f) * (Fm2 - Fm1 + Fm4 - Fm3);

%% Conditions initiales et intégration
x0 = [0; 0];   y0 = [0; 0];   z0 = [0; 0];
% tspan = [0 2];
% 
% [t, x] = ode45(@(t,x) roll_dynamics(t, x, Torque_roll, I_xx, b_roll), tspan, x0);
% [t, y] = ode45(@(t,y) pitch_dynamics(t, y, Torque_pitch, I_yy, b_pitch), tspan, y0);
% [t, z] = ode45(@(t,z) yaw_dynamics(t, z, Torque_yaw, I_zz, b_yaw), tspan, z0);
t_eval = linspace(0, 2, 200);   % 200 instants fixes, communs aux 3 axes

[~, x] = ode45(@(t,x) roll_dynamics(t, x, Torque_roll, I_xx, b_roll), t_eval, x0);
[~, y] = ode45(@(t,y) pitch_dynamics(t, y, Torque_pitch, I_yy, b_pitch), t_eval, y0);
[t, z] = ode45(@(t,z) yaw_dynamics(t, z, Torque_yaw, I_zz, b_yaw), t_eval, z0);

% %% Affichage
% figure;
% subplot(3,1,1); plot(t, rad2deg(x(:,1))); xlabel('Temps (s)'); ylabel('Roll (deg)');
% subplot(3,1,2); plot(t, rad2deg(y(:,1))); xlabel('Temps (s)'); ylabel('Pitch (deg)');
% subplot(3,1,3); plot(t, rad2deg(z(:,1))); xlabel('Temps (s)'); ylabel('Yaw (deg)');

%% Affichage complet : angles ET vitesses angulaires
figure;

subplot(3,2,1); plot(t, rad2deg(x(:,1))); ylabel('Roll (deg)'); title('Angles');
subplot(3,2,3); plot(t, rad2deg(y(:,1))); ylabel('Pitch (deg)');
subplot(3,2,5); plot(t, rad2deg(z(:,1))); ylabel('Yaw (deg)'); xlabel('Temps (s)');

subplot(3,2,2); plot(t, rad2deg(x(:,2))); ylabel('Roll rate (deg/s)'); title('Vitesses angulaires');
subplot(3,2,4); plot(t, rad2deg(y(:,2))); ylabel('Pitch rate (deg/s)');
subplot(3,2,6); plot(t, rad2deg(z(:,2))); ylabel('Yaw rate (deg/s)'); xlabel('Temps (s)');

%% Fonctions de dynamique (une par axe, structure identique)
function dxdt = roll_dynamics(t, x, Torque_roll, I, b_roll)
    angle_roll = x(1);
    omega_roll = x(2);
    Torque_net_roll = Torque_roll - b_roll * omega_roll;
    d_angle_roll = omega_roll;
    d_omega_roll = Torque_net_roll / I;
    dxdt = [d_angle_roll; d_omega_roll];
end

function dydt = pitch_dynamics(t, y, Torque_pitch, I, b_pitch)
    angle_pitch = y(1);
    omega_pitch = y(2);
    Torque_net_pitch = Torque_pitch - b_pitch * omega_pitch;
    d_angle_pitch = omega_pitch;
    d_omega_pitch = Torque_net_pitch / I;
    dydt = [d_angle_pitch; d_omega_pitch];
end

function dzdt = yaw_dynamics(t, z, Torque_yaw, I, b_yaw)
    angle_yaw = z(1);
    omega_yaw = z(2);
    Torque_net_yaw = Torque_yaw - b_yaw * omega_yaw;
    d_angle_yaw = omega_yaw;
    d_omega_yaw = Torque_net_yaw / I;
    dzdt = [d_angle_yaw; d_omega_yaw];
end