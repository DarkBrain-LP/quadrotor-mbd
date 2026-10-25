% motor_dist_from_center = 0.5;
% d = motor_dist_from_center;
% k = k_m / k_f;
% 
% % matrice de mixage du moteur
% servo_mixer =   [1   1   1   1;
%                  d  -d  -d   d;
%                 -d  -d   d   d;
%                 -k   k  -k   k];
% 
% TRPY = servo_mixer * [Fm1; Fm2; Fm3; Fm4];
% 
% Kp_alt = 0.4;
% Ki_alt = 0.06;
% Kb_alt = Ki_alt;
% 

%% ===================== PARAMÈTRES PHYSIQUES (TODO : update avec le vrai matériel) =====================
motor_dist_from_center = 0.11;   % VÉRIFIER : bras de levier réel, mètres "
d = motor_dist_from_center;

k_m = 0.02;    % coefficient de couple de réaction — à remplacer (datasheet hélice / banc de test)
k_f = 1.0;     % coefficient de poussée — à remplacer
k = k_m / k_f;

I_xx = 0.02;   % moment d'inertie roll, kg.m^2 — à remesurer une fois assemblé
I_yy = 0.02;   % moment d'inertie pitch, kg.m^2
I_zz = 0.02;   % moment d'inertie yaw, kg.m^2
I = [I_xx; I_yy; I_zz];   % vecteur [roll; pitch; yaw]

Fm_max = 2.5;  % poussée max par moteur, Newtons — à remplacer (datasheet moteur/hélice choisi)

%% ===================== MATRICE DE MIXAGE MOTEUR =====================
servo_mixer = [1   1   1   1;
               d  -d  -d   d;
              -d  -d   d   d;
              -k   k  -k   k];

% Vérification de cohérence : la matrice doit être inversible
if abs(det(servo_mixer)) < 1e-6
    warning('servo_mixer est (quasi) singulière — vérifier d et k, les axes ne seront pas découplés correctement.')
end

%% ===================== COUPLE MAX DISPONIBLE PAR AXE (pour la saturation du PID) =====================
Torque_roll_max  = d * (2*Fm_max);
Torque_pitch_max = d * (2*Fm_max);
Torque_yaw_max   = k * (2*Fm_max);
Torque_max = [Torque_roll_max; Torque_pitch_max; Torque_yaw_max];   % vecteur [roll; pitch; yaw]

%% Coefficient d'amortissement aérodynamique — désactivé (hypothèse simplifiée documentée)
b_roll = 0;
b_pitch = 0;
b_yaw = 0;

%% ===================== RÉGLAGE AUTOMATIQUE DU PID RATE =====================
tau = 0.05;          % constante de temps visée (boucle rate), secondes — choix de conception
facteur_Ki = 6.7;     % Ki = Kp / facteur_Ki (point de départ empirique)

Kp_rate = I ./ tau;                 % vecteur [Kp_roll; Kp_pitch; Kp_yaw]
Ki_rate = Kp_rate ./ facteur_Ki;    % vecteur [Ki_roll; Ki_pitch; Ki_yaw]
Kb_rate = Ki_rate;                  % règle de départ anti-windup

erreur_max_avant_saturation = Torque_max ./ Kp_rate;   % rad/s, vecteur [roll; pitch; yaw]

%% ===================== AFFICHAGE DE VÉRIFICATION =====================
disp('--- Géométrie ---')
fprintf('d = %.4f m,  k = %.4f\n', d, k)
disp('--- Gains PID rate [roll; pitch; yaw] ---')
disp(table(Kp_rate, Ki_rate, Kb_rate, Torque_max, erreur_max_avant_saturation, ...
    'RowNames', {'roll','pitch','yaw'}))

%% ===================== RÉGLAGE AUTOMATIQUE DU PID ANGLE =====================
tau_angle = 5*tau;          % constante de temps visée - empirique [3 - 10]x plus grand que tau_rate

Kp_angle = [1; 1; 1] ./ tau_angle;                 % vecteur [Kp_angle_roll; Kp_angle_pitch; Kp_angle_yaw]
Ki_angle = Kp_angle ./ facteur_Ki;    % vecteur [Ki_angle_roll; Ki_angle_pitch; Ki_angle_yaw]
Kb_angle = Ki_angle;                  % règle de départ anti-windup

omega_max = erreur_max_avant_saturation;
