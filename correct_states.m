%   Apply corrections after an EKF measurement update

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

%--> Apply updates
b_value(index+1) = b_value(index+1) + state_correction_vector(7);
c_value(index+1) = c_value(index+1) + state_correction_vector(8);
d_value(index+1) = d_value(index+1) + state_correction_vector(9);
a_value(index+1) = a_value(index+1) - ([b_value(index) c_value(index) d_value(index)]*state_correction_vector(7:9))/a_value(index);

gyro_bias_x_value(index+1) = gyro_bias_x_value(index+1) + state_correction_vector(10);
gyro_bias_y_value(index+1) = gyro_bias_y_value(index+1) + state_correction_vector(11);
gyro_bias_z_value(index+1) = gyro_bias_z_value(index+1) + state_correction_vector(12);
gyro_sf_x_value(index+1) = gyro_sf_x_value(index+1) + state_correction_vector(13);
gyro_sf_y_value(index+1) = gyro_sf_y_value(index+1) + state_correction_vector(14);
gyro_sf_z_value(index+1) = gyro_sf_z_value(index+1) + state_correction_vector(15);

acc_bias_x_value(index+1) = acc_bias_x_value(index+1) + state_correction_vector(16);
acc_bias_y_value(index+1) = acc_bias_y_value(index+1) + state_correction_vector(17);
acc_bias_z_value(index+1) = acc_bias_z_value(index+1) + state_correction_vector(18);
acc_sf_x_value(index+1) = acc_sf_x_value(index+1) + state_correction_vector(19);
acc_sf_y_value(index+1) = acc_sf_y_value(index+1) + state_correction_vector(20);
acc_sf_z_value(index+1) = acc_sf_z_value(index+1) + state_correction_vector(21);

%--> Normalize the quaternion
[a_value(index+1), b_value(index+1), c_value(index+1), d_value(index+1)] = ...
    normalize_quaternion([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)]);

%--> Calculate DCM from quaternion
C_LB_from_quat_value = C_LB_from_quat_fun(...
    b_value(index+1),...
    c_value(index+1),...
    d_value(index+1));

%--> Calculate Euler angles from DCM
[Euler_roll_value(index+1), Euler_pitch_value(index+1), Euler_heading_value(index+1)] = get_euler_from_dcm(C_LB_from_quat_value);

%--> Update position (not really needed since this is stationary AHRSscenario)
Rn_value(index+1)  = earth_a/sqrt(1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R));
Rm_value(index+1)  = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R))^(1.5));
EP_value(index+1) = (lon_value(index+1)-lon_value(1))*D2R*(Rn_value(index+1)+alt_value(index+1))*cos(lat_value(index+1)*D2R);
NP_value(index+1) = (lat_value(index+1)-lat_value(1))*D2R*(Rm_value(index+1)+alt_value(index+1));