motor_dist_from_center = 0.5;
d = motor_dist_from_center;
k = k_m / k_f;

% matrice de mixage du moteur
servo_mixer =   [1   1   1   1;
                 d  -d  -d   d;
                -d  -d   d   d;
                -k   k  -k   k];

TRPY = servo_mixer * [Fm1; Fm2; Fm3; Fm4];