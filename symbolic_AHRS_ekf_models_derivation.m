%   Symbolic definistions of system dynamics, error models, and EKF models

%   This is an Attitude and Heading Refrence System (AHRS) using IMU
%   This is a stationary AHRS scenario with attitude calculation only.
%   Velocity, and psoition calculation is not given in this version.

%   Copyright (C) 2018, Mohamed Atia, all rights reserved.
%   The software is given under GNU Lesser General Public License
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.

%   For commercial use, please contact mohamed.atia@carleton.ca

%--> Symbols definitions
reset(symengine);
syms    b c d;
%--> Gyro measurements and errors
syms    gyro_x gyro_y gyro_z;
syms    gyro_rw_stdv_x gyro_rw_stdv_y gyro_rw_stdv_z;
syms    gyro_bias_x gyro_bias_y gyro_bias_z;
syms    gyro_bias_gauss_markov_stdv_x gyro_bias_gauss_markov_stdv_y gyro_bias_gauss_markov_stdv_z;
syms    gyro_bias_x_time_cnst gyro_bias_y_time_cnst gyro_bias_z_time_cnst;
syms    gyro_sf_x gyro_sf_y gyro_sf_z;
syms    gyro_sf_gauss_markov_stdv_x gyro_sf_gauss_markov_stdv_y gyro_sf_gauss_markov_stdv_z;
syms    gyro_sf_x_time_cnst gyro_sf_y_time_cnst gyro_sf_z_time_cnst;

%-- Accel measurements and errors
syms    a_x a_y a_z;
syms    acc_rw_stdv_x acc_rw_stdv_y acc_rw_stdv_z;
syms    acc_bias_x acc_bias_y acc_bias_z;
syms    acc_bias_gauss_markov_stdv_x acc_bias_gauss_markov_stdv_y acc_bias_gauss_markov_stdv_z;
syms    acc_bias_x_time_cnst acc_bias_y_time_cnst acc_bias_z_time_cnst;
syms    acc_sf_x acc_sf_y acc_sf_z;
syms    acc_sf_gauss_markov_stdv_x acc_sf_gauss_markov_stdv_y acc_sf_gauss_markov_stdv_z;
syms    acc_sf_x_time_cnst acc_sf_y_time_cnst acc_sf_z_time_cnst;

syms    w_el_x w_el_y w_el_z;
syms    wg_noise 
syms    lat lon alt 
syms    wander_angle 
syms    w_N_IE we 
syms    vn ve vu 
syms    b_pos c_pos d_pos
syms    Rm Rn g lat0 lon0 
syms    ve_dot vn_dot vu_dot 
syms    Euler_pitch Euler_roll Euler_heading
syms    mag_x mag_y mag_z mag_bias_x mag_bias_y mag_bias_z 
syms    mag_rw_stdv_x mag_rw_stdv_y mag_rw_stdv_z
syms    a_pos_initial_error_covariance 
syms    b_pos_initial_error_covariance
syms    c_pos_initial_error_covariance
syms    d_pos_initial_error_covariance
syms    alt_initial_error_covariance 
syms    ve_initial_error_covariance
syms    vn_initial_error_covariance
syms    vu_initial_error_covariance
syms    a_initial_error_covariance 
syms    b_initial_error_covariance 
syms    c_initial_error_covariance
syms    d_initial_error_covariance
syms    gyro_bias_x_initial_error_covariance
syms    gyro_bias_y_initial_error_covariance
syms    gyro_bias_z_initial_error_covariance
syms    acc_bias_x_initial_error_covariance 
syms    acc_bias_y_initial_error_covariance
syms    acc_bias_z_initial_error_covariance
syms    a_pos_rw_stdv
syms    b_pos_rw_stdv
syms    c_pos_rw_stdv 
syms    d_pos_rw_stdv
syms    alt_rw_stdv
syms    east_pos north_pos

%--> Degrees to Radiants
D2R = pi/180;
R2D = 180/pi;

%--> Definitions
g_N = [0; 0; -g];                
V_N = [ve; vn; vu];

%--> raw measurements vectors
acc_B = [a_x/(1+acc_sf_x)-acc_bias_x+acc_rw_stdv_x*wg_noise;
         a_y/(1+acc_sf_y)-acc_bias_y+acc_rw_stdv_y*wg_noise;
         a_z/(1+acc_sf_z)-acc_bias_z+acc_rw_stdv_z*wg_noise];
     
gyro_B = [gyro_x/(1+gyro_sf_x)-gyro_bias_x+gyro_rw_stdv_x*wg_noise;
         gyro_y/(1+gyro_sf_y)-gyro_bias_y+gyro_rw_stdv_y*wg_noise;
         gyro_z/(1+gyro_sf_z)-gyro_bias_z+gyro_rw_stdv_z*wg_noise];

% acc_B = [(a_x-acc_bias_x)/(1+acc_sf_x)+acc_rw_stdv_x*wg_noise;
%          (a_y-acc_bias_y)/(1+acc_sf_y)+acc_rw_stdv_y*wg_noise;
%          (a_z-acc_bias_z)/(1+acc_sf_z)+acc_rw_stdv_z*wg_noise];
%      
% gyro_B = [(gyro_x-gyro_bias_x)/(1+gyro_sf_x)+gyro_rw_stdv_x*wg_noise;
%          (gyro_y-gyro_bias_y)/(1+gyro_sf_y)+gyro_rw_stdv_y*wg_noise;
%          (gyro_z-gyro_bias_z)/(1+gyro_sf_z)+gyro_rw_stdv_z*wg_noise];
     
mag_B = [mag_x-mag_bias_x+mag_rw_stdv_x*wg_noise;
         mag_y-mag_bias_y+mag_rw_stdv_y*wg_noise;
         mag_z-mag_bias_z+mag_rw_stdv_z*wg_noise];
     
%--> Frames transformation
C_NL = [0 1 0;
        1 0 0;
        0 0 -1];
C_LN = C_NL';

%--> General transformation from N to E with wander angle
D(1,1) = cos(lon)*cos(wander_angle) - sin(lon)*sin(lat)*sin(wander_angle);
D(1,2) = -cos(lon)*sin(wander_angle) - sin(lon)*sin(lat)*cos(wander_angle);
D(1,3) = sin(lon)*cos(lat);
D(2,1) = cos(lat)*sin(wander_angle);
D(2,2) = cos(lat)*cos(wander_angle);
D(2,3) = sin(lat);
D(3,1) = -sin(lon)*cos(wander_angle) - cos(lon)*sin(lat)*sin(wander_angle);
D(3,2) = sin(lon)*sin(wander_angle)-cos(lon)*sin(lat)*cos(wander_angle);
D(3,3) = cos(lon)*cos(lat);

%--> This implementation follows "free wander angle (wander_angle = 0)"
C_EN = subs(D,wander_angle,0);
C_NE = C_EN.';
C_EL = C_EN*C_NL;

%--> Quaternion Normalization Constraints
a       = sqrt(1- b^2 - c^2 - d^2);
a_pos   = sqrt(1 - b_pos^2 - c_pos^2 - d_pos^2);

%--> Attitude DCM in terms of quaternion
C_LB_from_quat = [(a^2+b^2-c^2-d^2)   2*(b*c-a*d)         2*(b*d+a*c);
                    2*(b*c+a*d)       (a^2-b^2+c^2-d^2)   2*(c*d-a*b);
                    2*(b*d-a*c)       2*(c*d + a*b)       (a^2-b^2-c^2+d^2)];

%--> Attitude DCM in terms of Euler
cp = cos(Euler_pitch);
ch = cos(Euler_heading);
sh = sin(Euler_heading);
cr = cos(Euler_roll);
sr = sin(Euler_roll);
sp = sin(Euler_pitch);

C_LB_from_Euler = [cp*ch   -cr*sh + sr*sp*ch    sr*sh + cr*sp*ch;
                   cp*sh    cr*ch + sr*sp*sh   -sr*ch + cr*sp*sh;
                   -sp      sr*cp               cr*cp];

%--> Transport rate 
w_N_EN = [-vn/(Rm+alt); ve/(Rn+alt); ve*tan(lat)/(Rn+alt)];
w_L_EN = C_LN*w_N_EN;
w_L_EL = [ ve/(Rn+alt); -vn/(Rm+alt);  ve*tan(lat)/(Rn+alt)];

%--> Earth rate
w_E_IE = [0; we; 0];
w_N_IE = C_NE*w_E_IE;
w_L_IE = C_LN*w_N_IE;

%--> Earth Rate + Transportation Rate compensation
w_N_IN = w_N_IE + w_N_EN;
w_L_IL = w_L_IE + w_L_EN;

w_il_x = w_L_IL(1);
w_il_y = w_L_IL(2);
w_il_z = w_L_IL(3);

%--> Attitude quaternion system equations
quat_vector      = [a;b;c;d];
att_quat_matrix  = [a -b -c -d;
                    b a -d c;
                    c d a -b;
                    d -c b a];
gyro_quat_vector = [0;gyro_B];

%--> Rate compensation quaternion vector
w_il_quat_matrix = [0           -w_il_x     -w_il_y     -w_il_z;...
                    w_il_x         0        -w_il_z      w_il_y;...
                    w_il_y       w_il_z        0        -w_il_x;...
                    w_il_z      -w_il_y      w_il_x        0   ];

%--> Attitude system equations
attitude_quat_dot = 0.5*att_quat_matrix*gyro_quat_vector - 0.5*w_il_quat_matrix*quat_vector;
a_dot = attitude_quat_dot(1);
b_dot = attitude_quat_dot(2);
c_dot = attitude_quat_dot(3);
d_dot = attitude_quat_dot(4);

%--> Gyro bias system equations          
gyro_bias_x_dot = -1/(gyro_bias_x_time_cnst)*gyro_bias_x + gyro_bias_gauss_markov_stdv_x/gyro_bias_x_time_cnst*wg_noise;
gyro_bias_y_dot = -1/(gyro_bias_y_time_cnst)*gyro_bias_y + gyro_bias_gauss_markov_stdv_y/gyro_bias_y_time_cnst*wg_noise;
gyro_bias_z_dot = -1/(gyro_bias_z_time_cnst)*gyro_bias_z + gyro_bias_gauss_markov_stdv_z/gyro_bias_z_time_cnst*wg_noise;

%Gyro scale factor system equations
gyro_sf_x_dot = -1/(gyro_sf_x_time_cnst)*gyro_sf_x + gyro_sf_gauss_markov_stdv_x/gyro_sf_x_time_cnst*wg_noise;
gyro_sf_y_dot = -1/(gyro_sf_y_time_cnst)*gyro_sf_y + gyro_sf_gauss_markov_stdv_y/gyro_sf_y_time_cnst*wg_noise;
gyro_sf_z_dot = -1/(gyro_sf_z_time_cnst)*gyro_sf_z + gyro_sf_gauss_markov_stdv_z/gyro_sf_z_time_cnst*wg_noise;

%--> Acc bias system equations          
acc_bias_x_dot = -1/(acc_bias_x_time_cnst)*acc_bias_x + acc_bias_gauss_markov_stdv_x/acc_bias_x_time_cnst*wg_noise;
acc_bias_y_dot = -1/(acc_bias_y_time_cnst)*acc_bias_y + acc_bias_gauss_markov_stdv_y/acc_bias_y_time_cnst*wg_noise;
acc_bias_z_dot = -1/(acc_bias_z_time_cnst)*acc_bias_z + acc_bias_gauss_markov_stdv_z/acc_bias_z_time_cnst*wg_noise;

%Acc scale factor system equations
acc_sf_x_dot = -1/(acc_sf_x_time_cnst)*acc_sf_x + acc_sf_gauss_markov_stdv_x/acc_sf_x_time_cnst*wg_noise;
acc_sf_y_dot = -1/(acc_sf_y_time_cnst)*acc_sf_y + acc_sf_gauss_markov_stdv_y/acc_sf_y_time_cnst*wg_noise;
acc_sf_z_dot = -1/(acc_sf_z_time_cnst)*acc_sf_z + acc_sf_gauss_markov_stdv_z/acc_sf_z_time_cnst*wg_noise;


%--> Velocity system equations
V_N_dot = C_NL * C_LB_from_quat * acc_B + g_N - cross((w_N_EN + 2*w_N_IE),V_N);
ve_dot = V_N_dot(1);
vn_dot = V_N_dot(2);
vu_dot = V_N_dot(3);

%--> Altituide System Equation:
alt_dot = vu + alt_rw_stdv*wg_noise;

%--> Horizontal Position calculated from velocities
lat_dot = vn/(Rm+alt);
lon_dot = ve/((Rn+alt)*cos(lat));

%--> Approximated east/north system equations
north_pos_dot = vn;
east_pos_dot = ve;

%--> Symbols lists
symbol_list             = [east_pos;north_pos;alt;...
                           ve;vn;vu;...
                           b;c;d;...
                           gyro_bias_x;gyro_bias_y;gyro_bias_z;...
                           gyro_sf_x;gyro_sf_y;gyro_sf_z;...
                           acc_bias_x;acc_bias_y;acc_bias_z;...
                           acc_sf_x;acc_sf_y;acc_sf_z;...
                           ];
                       
symbol_derivative_list  = [east_pos_dot;north_pos_dot;alt_dot;...
                           ve_dot;vn_dot;vu_dot;...
                           b_dot;c_dot;d_dot;...
                           gyro_bias_x_dot;gyro_bias_y_dot;gyro_bias_z_dot;...
                           gyro_sf_x_dot;gyro_sf_y_dot;gyro_sf_z_dot;...
                           acc_bias_x_dot;acc_bias_y_dot;acc_bias_z_dot;...
                           acc_sf_x_dot;acc_sf_y_dot;acc_sf_z_dot];

%--> Jacobian of the nonlinear system transition matrix (F)
for i = 1:length(symbol_list)
    for j = 1:length(symbol_list)
        F(i,j) = [diff(symbol_derivative_list(i), symbol_list(j))];
    end
end

%--> Jacobian of the noise shaping vector (G)
for j = 1:length(symbol_list)
    G(j,1) = diff(symbol_derivative_list(j), wg_noise);
end

%--> Gravity measurement model
quat_projection_matrix = [(a^2+b^2+c^2+d^2) 0                 0                 0;
                               0            (a^2+b^2-c^2-d^2) 2*(b*c-a*d)       2*(b*d+a*c);
                               0            2*(b*c+a*d)       (a^2-b^2+c^2-d^2) 2*(c*d-a*b);
                               0            2*(b*d-a*c)       2*(c*d + a*b)     (a^2-b^2-c^2+d^2)];
                           
acc_quat_vector = [0;acc_B];
g_L_meas_vector = quat_projection_matrix*acc_quat_vector;
for r = 1:length(g_L_meas_vector)
    for c = 1:length(symbol_list)
        g_L_meas_matrix(r,c) = [diff(g_L_meas_vector(r), symbol_list(c))];
    end
end

%--> Magnetic measurement model
mag_quat_vector = [0;mag_B];
m_L_meas_vector = quat_projection_matrix*mag_quat_vector;
for r = 1:length(g_L_meas_vector)
    for c = 1:length(symbol_list)
        m_L_meas_matrix(r,c) = [diff(m_L_meas_vector(r), symbol_list(c))];
    end
end
