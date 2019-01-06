%   Main module

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

% --> Initialize workspace settings
set(0,'DefaultFigureWindowStyle','docked');
clear;
close all;
fclose all;
clc;
if (~isempty(instrfind))
    fclose(instrfind);
end

logs_type   = 2; %(0 real static data, 1: real dynamic data, 2: simulated data)
EKF_mode    = 1; %(1 loosely-coupled, 2: tightly-coupled)
display_figures = 1;

%--> Generate symbolic models
symbolic_AHRS_ekf_models_derivation;
num_of_ekf_states = length(symbol_list);

%--> Get matlab function from symbolic models
C_LB_from_Euler_fun         = matlabFunction(C_LB_from_Euler);
w_L_IL_fun                  = matlabFunction(w_L_IL);
a_dot_fun                   = matlabFunction(a_dot);
b_dot_fun                   = matlabFunction(b_dot);
c_dot_fun                   = matlabFunction(c_dot);
d_dot_fun                   = matlabFunction(d_dot);
C_LB_from_quat_fun          = matlabFunction(C_LB_from_quat);
C_EN_fun                    = matlabFunction(C_EN);
C_EL_fun                    = matlabFunction(C_EL);
w_N_EN_fun                  = matlabFunction(w_N_EN);
w_N_IE_fun                  = matlabFunction(w_N_IE);
V_N_dot_fun                 = matlabFunction(V_N_dot);
w_L_IE_fun                  = matlabFunction(w_L_IE);
F_fun                       = matlabFunction(F);
G_fun                       = matlabFunction(G);
g_L_meas_vector_fun         = matlabFunction(g_L_meas_vector);
m_L_meas_vector_fun         = matlabFunction(m_L_meas_vector);
g_L_meas_matrix_fun         = matlabFunction(g_L_meas_matrix);
m_L_meas_matrix_fun         = matlabFunction(m_L_meas_matrix);

%--> Earth ellipsoid shape parameters
earth_a           = 6378137;
earth_f           = 1/298.257223563;
earth_b           = earth_a*(1-earth_f);
earth_e2          = 1-(earth_b^2)/(earth_a^2);
we_value          = 2*pi/(24*60*60);

%--> gravity constants
gravity_a1 = 9.7803267714;  a4_gravity =-0.000003087691089;
gravity_a2 = 0.0052790414;  a5_gravity = 0.000000004397731;
gravity_a3 = 0.0000232718;  a6_gravity = 0.000000000000721;

if logs_type == 2
    g_value(1) = 9.7934;
else
    g_value(1) = 9.8;
end

%--> Magnetometer calibration parameters
comp = [0.9851    0.0072   -0.0122;...
    0.0072    0.9879   -0.0000;...
    -0.0122   -0.0000    0.9860];

e_center = [-9.2819;12.0480;-14.2015];

%--> Initialize orientation
if logs_type == 2
    Euler_roll_value(1)     = 0.0;
    Euler_pitch_value(1)    = 0.0;
    Euler_heading_value(1)  = -15.0*D2R;
elseif logs_type == 0
    %---- for static test
    Euler_roll_value(1)     = 0.0;
    Euler_pitch_value(1)    = 0.0;
    Euler_heading_value(1)  = 0.0*D2R;
elseif logs_type == 1
    %--- for dynamic test
    Euler_roll_value(1)     = 1.767*D2R;
    Euler_pitch_value(1)    = -0*2374*D2R;
    Euler_heading_value(1)  = -17*D2R;
end

%--> Initial Direction Cosine Matrix (DCM)
C_LB_value = C_LB_from_Euler_fun(Euler_roll_value(1),Euler_pitch_value(1),Euler_heading_value(1));

%--> Initial Quaternion
attitude_quat_vector = get_quat_from_dcm(C_LB_value);
a_value(1) = attitude_quat_vector(1);
b_value(1) = attitude_quat_vector(2);
c_value(1) = attitude_quat_vector(3);
d_value(1) = attitude_quat_vector(4);

%--> Initialize DCM from Quaternion
C_LB_from_quat_value = C_LB_from_quat_fun(...
    b_value(1),...
    c_value(1),...
    d_value(1));

%--> Initalize Coordinates to Draw a 3D Orientation Cube
X = 20;Y = 10;Z = 6;
origin = [X/2 Y/2 Z/2];
initial_vert = [X Y 0; %(1)
    0 Y 0; %(2)
    0 Y Z; %(3)
    X Y Z; %(4)
    0 0 Z; %(5)
    X 0 Z; %(6)
    X 0 0; %(7)
    0 0 0];%(8)
%--> Center the cube around origin
for p = 1:8
    initial_vert(p,:) = initial_vert(p,:) - origin;
end
CubePoints = [initial_vert(:,1),initial_vert(:,2),initial_vert(:,3)];
[ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(initial_vert);
faces = [1 2 3 4; 4 3 5 6; 6 7 8 5; 1 2 8 7; 6 7 1 4; 2 3 5 8];

%--> Initial Position
if logs_type == 2
    lat_value(1)                = -32.8308;
    lon_value(1)                = -68.7928;
    alt_value(1)                = 700.0; % Ideally should come from a barometer  or any Altimeter
    magnetic_declination_angle  = -0.463; % calculated from http://geomag.nrcan.gc.ca/calc/mdcal-en.php
    ref_magnetic_field_in_L     = [0;19372.4; -144.9; -13060.0]./1000';%uT;
elseif logs_type == 0
    lat_value(1)                = 45.3843088;
    lon_value(1)                = -75.6967712;
    alt_value(1)                = 100.0; % Ideally should come from a barometer  or any Altimeter
    magnetic_declination_angle  = -13.313; % calculated from http://geomag.nrcan.gc.ca/calc/mdcal-en.php
    ref_magnetic_field_in_L     = [0; 17.669; -4.179; 50.989];
elseif logs_type == 1
    lat_value(1) =  43.6521570000;
    lon_value(1) = -79.3791450000;
    alt_value(1) = 100.0;
    magnetic_declination_angle  = 0.0;
    ref_magnetic_field_in_L     = [0; 19372.4; -144.9; -13060.0]./1000';
end

%--> Earth curvature parameters
Rn_value(1) = earth_a/sqrt(1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R));
Rm_value(1) = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R))^(1.5));
EP_value(1) = 0;
NP_value(1) = 0;

%--> Initial Position Quaternion
C_EN_value      = C_EN_fun(lat_value(1)*D2R , lon_value(1)*D2R);
pos_quat_vector = get_quat_from_dcm (C_EN_value);
a_pos_value(1)  = pos_quat_vector(1)/sqrt(sum(pos_quat_vector.^2));
b_pos_value(1)  = pos_quat_vector(2)/sqrt(sum(pos_quat_vector.^2));
c_pos_value(1)  = pos_quat_vector(3)/sqrt(sum(pos_quat_vector.^2));
d_pos_value(1)  = pos_quat_vector(4)/sqrt(sum(pos_quat_vector.^2));

%--> Initial velocity
ve_value(1) = 0.0;
vn_value(1) = 0.0;
vu_value(1) = 0.0;

%--> Position system noise (set to zero so we predict mean of states without noise)
a_pos_rw_stdv_value(1)  = 0.0;
b_pos_rw_stdv_value(1)  = 0.0;
c_pos_rw_stdv_value(1)  = 0.0;
d_pos_rw_stdv_value(1)  = 0.0;
alt_rw_stdv_value(1)    = 0.0;

%--> Random walk parameter (set to zero so we predict mean of states without noise)
wg_noise_value(1) = 0.0;

%--> Initial biases and scale factors
gyro_bias_x_value(1) = 0.0;
gyro_bias_y_value(1) = 0.0;
gyro_bias_z_value(1) = 0.0;
acc_bias_x_value(1)  = 0.0;
acc_bias_y_value(1)  = 0.0;
acc_bias_z_value(1)  = 0.0;
gyro_sf_x_value(1) = 0.0;
gyro_sf_y_value(1) = 0.0;
gyro_sf_z_value(1) = 0.0;
acc_sf_x_value(1)  = 0.0;
acc_sf_y_value(1)  = 0.0;
acc_sf_z_value(1)  = 0.0;

%--> Sampling period
if logs_type == 2
    %--- for simulation test
    sampling_period_sec = 0.01;
elseif logs_type == 0
    sampling_period_sec = 0.01;
elseif logs_type == 1
    %--- for dynamic test
    sampling_period_sec = 0.05;
end

%--> EKF Parameters
% (Acc Noise Parameters)
if logs_type == 2
    acc_rw_stdv_x_value(1) = 0.5;
    acc_rw_stdv_y_value(1) = 0.5;
    acc_rw_stdv_z_value(1) = 0.5;
    
    acc_bias_gauss_markov_stdv_x_value (1) = 0.0048;
    acc_bias_gauss_markov_stdv_y_value (1) = 0.0048;
    acc_bias_gauss_markov_stdv_z_value (1) = 0.0048;
    acc_bias_x_time_cnst_value(1) = 100;
    acc_bias_y_time_cnst_value(1) = 100;
    acc_bias_z_time_cnst_value(1) = 100;
    
    acc_sf_gauss_markov_stdv_x_value (1) = 1e-10;
    acc_sf_gauss_markov_stdv_y_value (1) = 1e-10;
    acc_sf_gauss_markov_stdv_z_value (1) = 1e-10;
    acc_sf_x_time_cnst_value(1) = 100;
    acc_sf_y_time_cnst_value(1) = 100;
    acc_sf_z_time_cnst_value(1) = 100;
    
elseif logs_type == 0
    acc_rw_stdv_x_value(1) = 0.5;
    acc_rw_stdv_y_value(1) = 0.5;
    acc_rw_stdv_z_value(1) = 0.5;
    
    acc_bias_gauss_markov_stdv_x_value (1) = 0.59*g_value(1);
    acc_bias_gauss_markov_stdv_y_value (1) = 0.60*g_value(1);
    acc_bias_gauss_markov_stdv_z_value (1) = 0.55*g_value(1);
    acc_bias_x_time_cnst_value(1) = 1.95*3600;
    acc_bias_y_time_cnst_value(1) = 1.95*3600;
    acc_bias_z_time_cnst_value(1) = 1.95*3600;
    
    acc_sf_gauss_markov_stdv_x_value (1) = .2;
    acc_sf_gauss_markov_stdv_y_value (1) = .2;
    acc_sf_gauss_markov_stdv_z_value (1) = .2;
    acc_sf_x_time_cnst_value(1) = 10;
    acc_sf_y_time_cnst_value(1) = 10;
    acc_sf_z_time_cnst_value(1) = 10;
    
elseif logs_type == 1
    acc_rw_stdv_x_value(1) = 0.5;
    acc_rw_stdv_y_value(1) = 0.5;
    acc_rw_stdv_z_value(1) = 0.5;
    
    acc_bias_gauss_markov_stdv_x_value (1) = 8.1338674856;
    acc_bias_gauss_markov_stdv_y_value (1) = 4.9683544614;
    acc_bias_gauss_markov_stdv_z_value (1) = 1.7274589943;
    acc_bias_x_time_cnst_value(1) = 3058;
    acc_bias_y_time_cnst_value(1) = 2688;
    acc_bias_z_time_cnst_value(1) = 1767;
    
    acc_sf_gauss_markov_stdv_x_value (1) = 1e-10;
    acc_sf_gauss_markov_stdv_y_value (1) = 1e-10;
    acc_sf_gauss_markov_stdv_z_value (1) = 1e-10;
    acc_sf_x_time_cnst_value(1) = 100;
    acc_sf_y_time_cnst_value(1) = 100;
    acc_sf_z_time_cnst_value(1) = 100;
end

if logs_type == 2
    gyro_rw_stdv_x_value(1) = 0.1;
    gyro_rw_stdv_y_value(1) = 0.1;
    gyro_rw_stdv_z_value(1) = 0.1;
    
    gyro_bias_gauss_markov_stdv_x_value (1) = 0.05*D2R;
    gyro_bias_gauss_markov_stdv_y_value (1) = 0.05*D2R;
    gyro_bias_gauss_markov_stdv_z_value (1) = 0.05*D2R;
    gyro_bias_x_time_cnst_value(1) = 100;
    gyro_bias_y_time_cnst_value(1) = 100;
    gyro_bias_z_time_cnst_value(1) = 100;
    
    gyro_sf_gauss_markov_stdv_x_value (1) = 1e-10;
    gyro_sf_gauss_markov_stdv_y_value (1) = 1e-10;
    gyro_sf_gauss_markov_stdv_z_value (1) = 1e-10;
    gyro_sf_x_time_cnst_value(1) = 100;
    gyro_sf_y_time_cnst_value(1) = 100;
    gyro_sf_z_time_cnst_value(1) = 100;
    
    % Measurement Noise
    R_roll              = 0.5;
    R_pitch             = 0.5;
    R_Az                = 0.5;
    
elseif logs_type == 0
    gyro_rw_stdv_x_value(1) = 0.1;
    gyro_rw_stdv_y_value(1) = 0.1;
    gyro_rw_stdv_z_value(1) = 0.1;
    
    gyro_bias_gauss_markov_stdv_x_value (1) = 3.881828867;
    gyro_bias_gauss_markov_stdv_y_value (1) = 0.53832221;
    gyro_bias_gauss_markov_stdv_z_value (1) = 1.05577565;
    gyro_bias_x_time_cnst_value(1) = 334;
    gyro_bias_y_time_cnst_value(1) = 3420;
    gyro_bias_z_time_cnst_value(1) = 2860;
    
    gyro_sf_gauss_markov_stdv_x_value (1) = .2;
    gyro_sf_gauss_markov_stdv_y_value (1) = .2;
    gyro_sf_gauss_markov_stdv_z_value (1) = .2;
    gyro_sf_x_time_cnst_value(1) = 10;
    gyro_sf_y_time_cnst_value(1) = 10;
    gyro_sf_z_time_cnst_value(1) = 10;
    
    gyro_stdv_x_nom         = 3.881828867;
    gyro_stdv_y_nom         = 0.53832221;
    gyro_stdv_z_nom         = 1.05577565;
    gyro_time_cnst_x_nom    = 334.1279294;
    gyro_time_cnst_y_nom    = 3420.141199;
    gyro_time_cnst_z_nom    = 2860.220363;
    
    % Measurement Noise
    R_roll              = 0.541155134;
    R_pitch             = 0.035345813;
    R_Az                = 0.038235859;
elseif logs_type == 1
    gyro_rw_stdv_x_value(1) = 0.1;
    gyro_rw_stdv_y_value(1) = 0.1;
    gyro_rw_stdv_z_value(1) = 0.1;
    
    gyro_bias_gauss_markov_stdv_x_value (1) = 4.4119170207;
    gyro_bias_gauss_markov_stdv_y_value (1) = 7.6975919847;
    gyro_bias_gauss_markov_stdv_z_value (1) = 9.4508976598;
    gyro_bias_x_time_cnst_value(1) = 246;
    gyro_bias_y_time_cnst_value(1) = 3305;
    gyro_bias_z_time_cnst_value(1) = 345;
    
    gyro_sf_gauss_markov_stdv_x_value (1) = 1e-10;
    gyro_sf_gauss_markov_stdv_y_value (1) = 1e-10;
    gyro_sf_gauss_markov_stdv_z_value (1) = 1e-10;
    gyro_sf_x_time_cnst_value(1) = 100;
    gyro_sf_y_time_cnst_value(1) = 100;
    gyro_sf_z_time_cnst_value(1) = 100;
    
    % Measurement Noise
    R_roll              = 8.3594562771;
    R_pitch             = 3.7128787894;
    R_Az                = 5.3556954880;
end

%--- Initial error states covariances
east_pos_error_covariance(1) = 10^2;
north_pos_error_covariance(1) = 10^2;
alt_error_covariance(1) = 10^2;
ve_error_covariance(1) = 10^2;
vn_error_covariance(1) = 10^2;
vu_error_covariance(1) = 10^2;
b_error_covariance(1) = 2^2;
c_error_covariance(1) = 2^2;
d_error_covariance(1) = 2^2;
gyro_bias_x_error_covariance(1) = 2^2;
gyro_bias_y_error_covariance(1) = 2^2;
gyro_bias_z_error_covariance(1) = 2^2;
gyro_sf_x_error_covariance(1) = 2^2;
gyro_sf_y_error_covariance(1) = 2^2;
gyro_sf_z_error_covariance(1) = 2^2;

acc_bias_x_error_covariance(1) = 2^2;
acc_bias_y_error_covariance(1) = 2^2;
acc_bias_z_error_covariance(1) = 2^2;
acc_sf_x_error_covariance(1) = 2^2;
acc_sf_y_error_covariance(1) = 2^2;
acc_sf_z_error_covariance(1) = 2^2;

%--> Initial covariance matrix (P)
P = diag([
    east_pos_error_covariance(1);
    north_pos_error_covariance(1);
    alt_error_covariance(1);
    ve_error_covariance(1);
    vn_error_covariance(1);
    vu_error_covariance(1);
    b_error_covariance(1);
    c_error_covariance(1);
    d_error_covariance(1);
    gyro_bias_x_error_covariance(1);
    gyro_bias_y_error_covariance(1);
    gyro_bias_z_error_covariance(1);
    gyro_sf_x_error_covariance(1);
    gyro_sf_y_error_covariance(1);
    gyro_sf_z_error_covariance(1);
    acc_bias_x_error_covariance(1);
    acc_bias_y_error_covariance(1);
    acc_bias_z_error_covariance(1);
    acc_sf_x_error_covariance(1);
    acc_sf_y_error_covariance(1);
    acc_sf_z_error_covariance(1)
    ]);

error = 0;
counter = 1;

%--> Initial Quasi-stationary roll/pitch angles
roll_quasi(1) = 0;
pitch_quasi(1) = 0;

%--> Read raw data file
if logs_type == 2
    load('simulated_ahrs_logs.mat');
elseif logs_type == 0
    load('physical_ahrs_logs.mat');
elseif logs_type == 1
    load('dynamic_ahrs_logs.mat');
end
num_of_records = length(raw_imu_data(:,1));
C_LB_value = C_LB_from_Euler_fun(raw_imu_data(1,11),raw_imu_data(1,12),raw_imu_data(1,13));
C_LB_value = normalize_dcm(C_LB_value);
q_ref(1,:) = get_quat_from_dcm(C_LB_value);
[q_ref(1,1), q_ref(1,2), q_ref(1,3), q_ref(1,4)] = normalize_quaternion(q_ref(1,:));

%--> Loop counters
line_counter = 0;
index = 1;
innovation_seq_counter = 1;

%--> Main processing loop
for index = 1:num_of_records-1
    %--> Read one data record
    raw_acc_x(index) = raw_imu_data(index,2)*g_value(index);
    raw_acc_y(index) = raw_imu_data(index,3)*g_value(index);
    raw_acc_z(index) = raw_imu_data(index,4)*g_value(index);
    
    raw_gyro_x(index) = raw_imu_data(index,5)*D2R;
    raw_gyro_y(index) = raw_imu_data(index,6)*D2R;
    raw_gyro_z(index) = raw_imu_data(index,7)*D2R;
    
    raw_mag_x(index) = raw_imu_data(index,9);
    raw_mag_y(index) = raw_imu_data(index,8);
    raw_mag_z(index) = -raw_imu_data(index,10);
    
    if logs_type == 2
        time_vector_sec(index) = raw_imu_data(index,1);
    elseif logs_type == 0
        %--- for static test
        time_vector_sec(index) = raw_imu_data(index,1)/1000.0;
    elseif logs_type == 1
        %--- for dynamic test
        time_vector_sec(index) = raw_imu_data(index,1);
    end
    %--> compensate for magnetometer distortions
    mag_corrected = comp * ([raw_mag_x(index);raw_mag_y(index);raw_mag_z(index)] - e_center);
    raw_mag_x(index) = mag_corrected(1);
    raw_mag_y(index) = mag_corrected(2);
    raw_mag_z(index) = mag_corrected(3);
    
    % Calculate Earth Rotation Rate, Transport Rate, and Corriollis effect
    w_N_EN_value = w_N_EN_fun(Rm_value(index),Rn_value(index),alt_value(index),lat_value(index)*D2R,ve_value(index),vn_value(index));
    w_N_IE_value = w_N_IE_fun(lat_value(index)*D2R,we_value);
    w_L_IL_value = C_LN*(w_N_EN_value + w_N_IE_value);
    w_B_IL_value = C_LB_from_quat_value'*w_L_IL_value;
    corriollis_effect_vector_in_B = C_LB_from_quat_value'*C_LN*cross((w_N_EN_value + 2*w_N_IE_value),[ve_value(index);vn_value(index);vu_value(index)]);
    
    
    % Perform attitude quaternion mechanization
    a_dot_value = a_dot_fun(Rm_value(index),Rn_value(index),alt_value(index),b_value(index),c_value(index),d_value(index),raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),0.0,0.0,0.0,lat_value(index)*D2R,ve_value(index),vn_value(index),we_value,wg_noise_value(index));
    b_dot_value = b_dot_fun(Rm_value(index),Rn_value(index),alt_value(index),b_value(index),c_value(index),d_value(index),raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),0.0,0.0,0.0,lat_value(index)*D2R,ve_value(index),vn_value(index),we_value,wg_noise_value(index));
    c_dot_value = c_dot_fun(Rm_value(index),Rn_value(index),alt_value(index),b_value(index),c_value(index),d_value(index),raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),0.0,0.0,0.0,lat_value(index)*D2R,ve_value(index),vn_value(index),we_value,wg_noise_value(index));
    d_dot_value = d_dot_fun(Rm_value(index),Rn_value(index),alt_value(index),b_value(index),c_value(index),d_value(index),raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),0.0,0.0,0.0,lat_value(index)*D2R,ve_value(index),vn_value(index),we_value,wg_noise_value(index));
    
    if (~isreal([a_dot_value b_dot_value c_dot_value d_dot_value]))
        disp('imaginary quaternion rate');return;
    end
    
    % Advance quaternion
    a_value(index+1) = a_value(index) + a_dot_value*sampling_period_sec;
    b_value(index+1) = b_value(index) + b_dot_value*sampling_period_sec;
    c_value(index+1) = c_value(index) + c_dot_value*sampling_period_sec;
    d_value(index+1) = d_value(index) + d_dot_value*sampling_period_sec;
    
    % Normalize the quaternion
    [a_value(index+1), b_value(index+1), c_value(index+1), d_value(index+1)] = ...
        normalize_quaternion([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)]);
    
    % Calculate DCM from Quaternion
    C_LB_from_quat_value = C_LB_from_quat_fun(...
        b_value(index+1),...
        c_value(index+1),...
        d_value(index+1));

    C_LB_value = C_LB_from_Euler_fun(raw_imu_data(index+1,11),raw_imu_data(index+1,12),raw_imu_data(index+1,13));
    C_LB_value = normalize_dcm(C_LB_value);
    q_ref(index+1,:) = get_quat_from_dcm(C_LB_value);
    [q_ref(index+1,1), q_ref(index+1,2), q_ref(index+1,3), q_ref(index+1,4)] = normalize_quaternion(q_ref(index+1,:));

    if (~isreal(C_LB_from_quat_value))
        disp('imaginary DCM C_LB');return;
    end
    
    % Normalize the DCM
    C_LB_from_quat_value = normalize_dcm(C_LB_from_quat_value);
    
    % Calculate Euler angles from DCM
    [Euler_roll_value(index+1), Euler_pitch_value(index+1), Euler_heading_value(index+1)] = get_euler_from_dcm(C_LB_from_quat_value);
    
    if (~isreal([Euler_roll_value(index+1) Euler_pitch_value(index+1) Euler_heading_value(index+1)]))
        disp('imaginary Euler angles');return;
    end
    
    %--> This is a stationary AHRS scenario. Velocity is set to zero.
    ve_value(index+1) = ve_value(index);
    vn_value(index+1) = vn_value(index);
    vu_value(index+1) = vu_value(index);
    
    %--> This is a stationary AHRS scenario. Position is not changing.
    lat_value(index+1) = lat_value(index);
    lon_value(index+1) = lon_value(index);
    alt_value(index+1) = alt_value(index);
    
    %--> This is a stationary AHRS scenario, no change in these values
    Rn_value(index+1)  = Rn_value(index);
    Rm_value(index+1)  = Rm_value(index);
    EP_value(index+1)  = EP_value(index);
    NP_value(index+1)  = NP_value(index);
    
    %--> Bias prediction (no change until an update is perfoemd)
    gyro_bias_x_value(index+1) = gyro_bias_x_value(index);
    gyro_bias_y_value(index+1) = gyro_bias_y_value(index);
    gyro_bias_z_value(index+1) = gyro_bias_z_value(index);
    acc_bias_x_value(index+1) = acc_bias_x_value(index);
    acc_bias_y_value(index+1) = acc_bias_y_value(index);
    acc_bias_z_value(index+1) = acc_bias_z_value(index);
    
    gyro_sf_x_value(index+1) = gyro_sf_x_value(index);
    gyro_sf_y_value(index+1) = gyro_sf_y_value(index);
    gyro_sf_z_value(index+1) = gyro_sf_z_value(index);
    acc_sf_x_value(index+1) = acc_sf_x_value(index);
    acc_sf_y_value(index+1) = acc_sf_y_value(index);
    acc_sf_z_value(index+1) = acc_sf_z_value(index);
    
    %--> Get heading from mag
    mag_heading(index)      = atan2( (-raw_mag_y(index)*cos(Euler_roll_value(index))+ raw_mag_z(index)*sin(Euler_roll_value(index)) ),...
        (raw_mag_x(index)*cos(Euler_pitch_value(index)) + ...
        raw_mag_y(index)*sin(Euler_roll_value(index))*sin(Euler_pitch_value(index)) + ...
        raw_mag_z(index)*cos(Euler_roll_value(index))*sin(Euler_pitch_value(index))));
    
    mag_true_north_heading(index) = mag_heading(index) + magnetic_declination_angle*D2R;
    
    %--> Kalman Filter model matrices calculation
    %--> F matrix
    F_value = F_fun(Rm_value(index),Rn_value(index),raw_acc_x(index),raw_acc_y(index),raw_acc_z(index),...
        acc_sf_x_value(index),acc_sf_y_value(index),acc_sf_y_value(index),...
        acc_bias_x_value(index),acc_bias_y_value(index),acc_bias_z_value(index),...
        acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
        acc_sf_x_time_cnst_value(index),acc_sf_y_time_cnst_value(index),acc_sf_z_time_cnst_value(index),...
        acc_bias_x_time_cnst_value(index),acc_bias_y_time_cnst_value(index),acc_bias_z_time_cnst_value(index),...
        alt_value(index),b_value(index),c_value(index),d_value(index),...
        raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),...
        gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),...
        gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),...
        gyro_rw_stdv_x_value(index),gyro_rw_stdv_y_value(index),gyro_rw_stdv_z_value(index),...
        gyro_sf_x_time_cnst_value(index),gyro_sf_y_time_cnst_value(index),gyro_sf_z_time_cnst_value(index),...
        gyro_bias_x_time_cnst_value(index),gyro_bias_y_time_cnst_value(index),gyro_bias_z_time_cnst_value(index),...
        lat_value(index)*R2D,ve_value(index),vn_value(index),vu_value(index),we_value,wg_noise_value(index));
    
    if (~isreal(F_value))
        disp('imaginary transition matrix');return;
    end
    
    %--> Phi matrix
    Phi = eye(size(F_value)) + F_value*sampling_period_sec;
    
    %--> G matrix
    G_value = G_fun(acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
        acc_sf_x_time_cnst_value(index),acc_sf_z_time_cnst_value(index),acc_sf_y_time_cnst_value(index),...
        acc_bias_x_time_cnst_value(index),acc_bias_y_time_cnst_value(index),acc_bias_z_time_cnst_value(index),...
        acc_sf_gauss_markov_stdv_x_value(index),acc_sf_gauss_markov_stdv_y_value(index),acc_sf_gauss_markov_stdv_z_value(index),...
        acc_bias_gauss_markov_stdv_x_value(index),acc_bias_gauss_markov_stdv_y_value(index),acc_bias_gauss_markov_stdv_z_value(index),...
        alt_rw_stdv_value(index),...
        b_value(index), c_value(index),d_value(index),...
        gyro_rw_stdv_x_value(index),gyro_rw_stdv_y_value(index),gyro_rw_stdv_z_value(index),...
        gyro_sf_x_time_cnst_value(index),gyro_sf_y_time_cnst_value(index),gyro_sf_z_time_cnst_value(index),...
        gyro_bias_x_time_cnst_value(index),gyro_bias_y_time_cnst_value(index),gyro_bias_z_time_cnst_value(index),...
        gyro_sf_gauss_markov_stdv_x_value(index),gyro_sf_gauss_markov_stdv_y_value(index),gyro_sf_gauss_markov_stdv_z_value(index),...
        gyro_bias_gauss_markov_stdv_x_value(index),gyro_bias_gauss_markov_stdv_y_value(index),gyro_bias_gauss_markov_stdv_z_value(index));
    
    if (~isreal(G_value))
        disp('imaginary noise shaping matrix');return;
    end
    
    %--> Qd matrix
    Qd = sampling_period_sec^2*G_value*G_value';
    
    % Update quasi-stationary roll/pitch
    if index <= 9
        roll_quasi(index+1)     = roll_quasi(index);
        pitch_quasi(index+1)    = pitch_quasi(index);
    else
        roll_quasi(index+1)     = atan2(-mean(raw_acc_y(index-9:index)),-mean(raw_acc_z(index-9:index)));
        pitch_quasi(index+1)    = atan2(mean(raw_acc_x(index-9:index)),sqrt(mean(raw_acc_y(index-9:index))^2+mean(raw_acc_z(index-9:index))^2));
    end
    
    %--> EKF prediction
    P = Phi*P*Phi' + Qd;
    
    if (~isreal(P))
        disp('imaginary noise covariance matrix');return;
    end
    
    %--> Apply update at 1Hz rate
    if (mod(index,(1/sampling_period_sec)) == 0)
        
        state_correction_vector = zeros(length(P),1);
        
        if(EKF_mode == 1)%Loosely-coupled
            
            %--> Attitude Update
            %--> Take roll/pitch updates from accelerometers
            roll_quasi(index+1)     = atan2(-mean(raw_acc_y(index-9:index)),-mean(raw_acc_z(index-9:index)));
            pitch_quasi(index+1)    = atan2(mean(raw_acc_x(index-9:index)),sqrt(mean(raw_acc_y(index-9:index))^2+mean(raw_acc_z(index-9:index))^2));
            
            %--> Take heading from update from mag
            Updated_C_LB_value      = C_LB_from_Euler_fun(roll_quasi(index+1),...
                pitch_quasi(index+1),...
                mag_true_north_heading(index));
            
            %--> Convert the updates from Euler to quaternion form
            updated_quat            = get_quat_from_dcm(Updated_C_LB_value);
            
            %--> Set the innovation sequence
            z                       = updated_quat(2:4) - [b_value(index+1); c_value(index+1); d_value(index+1)];
            
            b_innovation_seq(innovation_seq_counter) = z(1);
            c_innovation_seq(innovation_seq_counter) = z(2);
            d_innovation_seq(innovation_seq_counter) = z(3);
            
            %--> Set design (measurement) matrix
            H                       = zeros(3,num_of_ekf_states);
            H(1,7)                  = 1;
            H(2,8)                  = 1;
            H(3,9)                  = 1;
            R                       = diag([R_roll,R_pitch,R_Az]);
        elseif (EKF_mode == 2) %Tightly-code
            % Gravity measurement update
            g_L_meas_vector_value       = g_L_meas_vector_fun(raw_acc_x(index),raw_acc_y(index),raw_acc_z(index),...
                acc_sf_x_value(index),acc_sf_y_value(index),acc_sf_z_value(index),...
                acc_bias_x_value(index),acc_bias_y_value(index),acc_bias_z_value(index),...
                acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
                b_value(index),c_value(index),d_value(index),wg_noise_value(index));
            
            g_L_meas_matrix_fun_value   = g_L_meas_matrix_fun(raw_acc_x(index),raw_acc_y(index),raw_acc_z(index),...
                acc_sf_x_value(index),acc_sf_y_value(index),acc_sf_z_value(index),...
                acc_bias_x_value(index),acc_bias_y_value(index),acc_bias_z_value(index),...
                acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
                b_value(index),c_value(index),d_value(index),wg_noise_value(index));
            
            zg                           = [0;0;0;-g_value(index)] - g_L_meas_vector_value;
            Hg                           = g_L_meas_matrix_fun_value;
            
            %--> Magnetic field measurement update
            m_L_meas_vector_value       = m_L_meas_vector_fun(b_value(index),c_value(index),d_value(index),...
                raw_mag_x(index),raw_mag_y(index),raw_mag_z(index),...
                0.0,0.0,0.0,...
                0.0,0.0,0.0,...
                0.0);
            
            m_L_meas_matrix_fun_value   = m_L_meas_matrix_fun(b_value(index),c_value(index),d_value(index),...
                raw_mag_x(index),raw_mag_y(index),raw_mag_z(index),...
                0.0,0.0,0.0,...
                0.0,0.0,0.0,...
                0.0);
            
            %--> Combined Z, and H matrices
            zm                           = ref_magnetic_field_in_L - m_L_meas_vector_value;
            Hm                           = m_L_meas_matrix_fun_value;
            
            %--> XY Acc/Mag Updates Only
            %             z = [zg(2:3);zm(2:3)];
            %             H = [Hg(2:3,:);Hm(2:3,:)];
            %             R = diag([2,2,5000,5000]);
            
            %--> YZ Acc Updates Only
            %             z = [zg(3:4)];
            %             H = [Hg(3:4,:)];
            %             R = diag([2,.2]);
            
            %--> YZ Acc and XY Mag Updates Only
            %             z = [zg(3:4);zm(2:3)];
            %             H = [Hg(3:4,:);Hm(2:3,:)];
            %             R = diag([2,.2,5000,5000]);
            
            %--> YZ Acc and YZ Mag Updates Only
            %             z = [zg(3:4);zm(3:4)];
            %             H = [Hg(3:4,:);Hm(3:4,:)];
            %             if logs_type == 0
            %                 R = diag([3,.3,30000,30000]);
            %             elseif logs_type == 2
            %                 R = diag([2,.2,5000,5000]);
            %             end
            
            %--> X Acc and Y Mag Updates Only
            %             z = [zg(2);zm(3)];
            %             H = [Hg(2,:);Hm(3,:)];
            %             if logs_type == 0
            %                 R = diag([1,500]);
            %             elseif logs_type == 2
            %                 R = diag([2,5000]); better for simulation data
            %             end
            
            %--> Y Acc and X Mag Updates Only
            %             z = [zg(3);zm(2)];
            %             H = [Hg(3,:);Hm(2,:)];
            %             if logs_type == 0
            %                 R = diag([1,500]);
            %             elseif logs_type == 2
            %                 R = diag([2,5000]); better for simulation data
            %             end
            
            %--> All Updates
            z = [zg(2:4);zm(2:4)];
            H = [Hg(2:4,:);Hm(2:4,:)];
            R = diag([2,2,.2,5000,5000,5000]);
            
        end
        
        %--> Calculate Kalman Gain
        K = P*H'/(H*P*H'+R);
        
        %--> Calculate error states vector
        state_correction_vector = state_correction_vector + K*z;
        
        %--> Update error cpvariance matrix
        P = P - K*H*P;
        clear z H R;
        
        if (~isreal(state_correction_vector))
            disp('imaginary error correction vector');return;
        end
        
        if (~isreal(P))
            disp('imaginary updated error covariance matrix ');return;
        end
        
        %--> Apply corrections
        correct_states;
    end
    
    %--> Normalize P
    P = (P+P')/2;
    
    if (~isreal(P))
        disp('imaginary error covariance matrix ');return;
    end
    
    %--> In system mean prediction, wg_noise is set to zero
    wg_noise_value(index+1) = 0.0;
    g_value(index + 1)      = g_value(index);
    
    %--> Record debugging information
    east_pos_error_covariance(index+1) = P(1,1);
    north_pos_error_covariance(index+1) = P(2,2);
    alt_error_covariance(index+1) = P(3,3);
    ve_error_covariance(index+1) = P(4,4);
    vn_error_covariance(index+1) = P(5,5);
    vu_error_covariance(index+1) = P(6,6);
    b_error_covariance(index+1) = P(7,7);
    c_error_covariance(index+1) = P(8,8);
    d_error_covariance(index+1) = P(9,9);
    gyro_bias_x_error_covariance(index+1) = P(10,10);
    gyro_bias_y_error_covariance(index+1) = P(11,11);
    gyro_bias_z_error_covariance(index+1) = P(12,12);
    acc_bias_x_error_covariance(index+1) = P(13,13);
    acc_bias_y_error_covariance(index+1) = P(14,14);
    acc_bias_z_error_covariance(index+1) = P(15,15);
    
    gyro_rw_stdv_x_value(index+1) = gyro_rw_stdv_x_value(index);
    gyro_rw_stdv_y_value(index+1) = gyro_rw_stdv_y_value(index);
    gyro_rw_stdv_z_value(index+1) = gyro_rw_stdv_z_value(index);
    gyro_bias_gauss_markov_stdv_x_value (index+1) = gyro_bias_gauss_markov_stdv_x_value (index);
    gyro_bias_gauss_markov_stdv_y_value (index+1) = gyro_bias_gauss_markov_stdv_y_value (index);
    gyro_bias_gauss_markov_stdv_z_value (index+1) = gyro_bias_gauss_markov_stdv_z_value (index);
    gyro_bias_x_time_cnst_value(index+1) = gyro_bias_x_time_cnst_value(index);
    gyro_bias_y_time_cnst_value(index+1) = gyro_bias_y_time_cnst_value(index);
    gyro_bias_z_time_cnst_value(index+1) = gyro_bias_z_time_cnst_value(index);
    
    gyro_sf_gauss_markov_stdv_x_value (index+1) = gyro_sf_gauss_markov_stdv_x_value (index);
    gyro_sf_gauss_markov_stdv_y_value (index+1) = gyro_sf_gauss_markov_stdv_y_value (index);
    gyro_sf_gauss_markov_stdv_z_value (index+1) = gyro_sf_gauss_markov_stdv_z_value (index);
    gyro_sf_x_time_cnst_value(index+1) = gyro_sf_x_time_cnst_value(index);
    gyro_sf_y_time_cnst_value(index+1) = gyro_sf_y_time_cnst_value(index);
    gyro_sf_z_time_cnst_value(index+1) = gyro_sf_z_time_cnst_value(index);
    
    acc_rw_stdv_x_value(index+1) = acc_rw_stdv_x_value(index);
    acc_rw_stdv_y_value(index+1) = acc_rw_stdv_y_value(index);
    acc_rw_stdv_z_value(index+1) = acc_rw_stdv_z_value(index);
    acc_bias_gauss_markov_stdv_x_value (index+1) = acc_bias_gauss_markov_stdv_x_value (index);
    acc_bias_gauss_markov_stdv_y_value (index+1) = acc_bias_gauss_markov_stdv_y_value (index);
    acc_bias_gauss_markov_stdv_z_value (index+1) = acc_bias_gauss_markov_stdv_z_value (index);
    acc_bias_x_time_cnst_value(index+1) = acc_bias_x_time_cnst_value(index);
    acc_bias_y_time_cnst_value(index+1) = acc_bias_y_time_cnst_value(index);
    acc_bias_z_time_cnst_value(index+1) = acc_bias_z_time_cnst_value(index);
    
    acc_sf_gauss_markov_stdv_x_value (index+1) = acc_sf_gauss_markov_stdv_x_value (index);
    acc_sf_gauss_markov_stdv_y_value (index+1) = acc_sf_gauss_markov_stdv_y_value (index);
    acc_sf_gauss_markov_stdv_z_value (index+1) = acc_sf_gauss_markov_stdv_z_value (index);
    acc_sf_x_time_cnst_value(index+1) = acc_sf_x_time_cnst_value(index);
    acc_sf_y_time_cnst_value(index+1) = acc_sf_y_time_cnst_value(index);
    acc_sf_z_time_cnst_value(index+1) = acc_sf_z_time_cnst_value(index);
    
    a_pos_rw_stdv_value(index + 1) = a_pos_rw_stdv_value(index);
    b_pos_rw_stdv_value(index + 1) = b_pos_rw_stdv_value(index);
    c_pos_rw_stdv_value(index + 1) = c_pos_rw_stdv_value(index);
    d_pos_rw_stdv_value(index + 1) = d_pos_rw_stdv_value(index);
    alt_rw_stdv_value(index + 1)   = alt_rw_stdv_value(index);
    
    % Disply status and results
    
    updated_vert = initial_vert*C_LB_from_quat_value;
    [ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(updated_vert);
    CubePoints = [updated_vert(:,1),updated_vert(:,2),updated_vert(:,3)];
    
    fprintf('record:%d/%d, time:%.10f\n',index,num_of_records,index*num_of_records);
    
    %     if display_figures
    %         if(index == 2)
    %             %--> Create plots for the first time
    %             create_figures;
    %         else
    %             %--> Refresh plots
    %             if (mod(index,(1/sampling_period_sec)) == 0)
    %                 refresh_figures;
    %             end
    %         end
    %     end
end

%--> calculate the error
roll_error = sqrt(mean( (raw_imu_data(:,11)-Euler_roll_value').^2 ))*R2D;
pitch_error = sqrt(mean( (raw_imu_data(:,12)-Euler_pitch_value').^2 ))*R2D;
heading_error = sqrt(mean( (raw_imu_data(:,13)-Euler_heading_value').^2 ))*R2D;
att_error = sqrt(heading_error^2+roll_error^2+pitch_error^2);
orientation_error = att_error;

fprintf('Roll Error (deg):%.10f\n',roll_error);
fprintf('Pitch Error (deg):%.10f\n',pitch_error);
fprintf('Azimuth Error (deg):%.10f\n',heading_error);
fprintf('3D Orientation Error (deg):%.10f\n',orientation_error);

create_figures;

fclose all;