%   Plots Creation

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

figure;
h1 = plot(time_vector_sec-time_vector_sec (1), Euler_heading_value(1,1:end-1)*R2D, 'r');hold on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,13)*R2D, '-','color','b','linewidth',1);grid on;
xlabel('time(s)');ylabel('Heading(deg)');hold on;grid on;
legend('EKF','Ground Truth');
title('Heading Estimation Results');

set(h1,'YDataSource','Euler_heading_value(1,1:end-1)*R2D');
set(h1,'XDataSource','time_vector_sec-time_vector_sec (1)');

figure;
h3 = plot(time_vector_sec-time_vector_sec (1), Euler_roll_value(1,1:end-1)*R2D, 'r');hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,11)*R2D, '-','color','b','linewidth',1);
xlabel('time(s)');ylabel('Roll(deg)');hold on;
legend('EKF','Ground Truth');
title('Roll Estimation Results');

figure;
h2 = plot(time_vector_sec-time_vector_sec (1), Euler_pitch_value(1,1:end-1)*R2D, 'r');hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,12)*R2D, '-','color','b','linewidth',1);grid on;
xlabel('time(s)');ylabel('Pitch(deg)');hold on;
legend('EKF','Ground Truth');
title('Pitch Estimation Results');

set(h2,'YDataSource','Euler_pitch_value(1,1:end-1)*R2D');
set(h2,'XDataSource','time_vector_sec-time_vector_sec (1)');
set(h3,'YDataSource','Euler_roll_value(1,1:end-1)*R2D');
set(h3,'XDataSource','time_vector_sec-time_vector_sec (1)');

figure;
h4 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_x*R2D, 'b');hold on;
h5 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_y*R2D, 'g');
h6 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_z*R2D, 'r');
legend('gyro x','gyro y','gyro z');grid on;xlabel('time(s)'); ylabel('raw gyro(deg/s)');
set(h4,'YDataSource','raw_gyro_x*R2D');
set(h4,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h5,'YDataSource','raw_gyro_y*R2D');
set(h5,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h6,'YDataSource','raw_gyro_z*R2D');
set(h6,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h13 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_x, 'b');hold on;
h14 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_y, 'g');
h15 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_z, 'r');
legend('acc x','acc y','acc z');grid on;xlabel('time(s)'); ylabel('raw acc(m/s2)');
set(h13,'YDataSource','raw_acc_x');
set(h13,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h14,'YDataSource',' raw_acc_y');
set(h14,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h15,'YDataSource','raw_acc_z');
set(h15,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h16 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_x, 'b');hold on;
h17 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_y, 'g');
h18 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_z, 'r');
legend('mag x','mag y','mag z');grid on;xlabel('time(s)'); ylabel('mag (uT)');
set(h16,'YDataSource','raw_mag_x');
set(h16,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h17,'YDataSource','raw_mag_y');
set(h17,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h18,'YDataSource','raw_mag_z');
set(h18,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h23 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_x_value(1:end-1)*R2D, 'k');grid on;hold on;
h24 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_y_value(1:end-1)*R2D, '-.k');grid on;hold on;
h25 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_z_value(1:end-1)*R2D, '--k');grid on;hold on;
time_vector = time_vector_sec(1:end)-time_vector_sec (1);
legend('gyro bias x','gyro bias y', 'gyro bias z');grid on;xlabel('time(s)'); ylabel('gyro bias (deg/s)');title('Gyro Bias(deg/s)');

figure;
Vertices(:,2) = [gyro_bias_x_value(1:end-1)'*R2D+2*sqrt(gyro_bias_x_error_covariance(1:end-1))'; flipdim(gyro_bias_x_value(1:end-1)'*R2D-2*sqrt(gyro_bias_x_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_x_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('stdv','gyro bias x');grid on;xlabel('time(s)'); ylabel('gyro x bias (deg/s)');title('Gyro Bias(deg/s)');

figure;
Vertices(:,2) = [gyro_bias_y_value(1:end-1)'*R2D+2*sqrt(gyro_bias_y_error_covariance(1:end-1))'; flipdim(gyro_bias_y_value(1:end-1)'*R2D-2*sqrt(gyro_bias_y_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_y_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('stdv','gyro bias y');grid on;xlabel('time(s)'); ylabel('gyro y bias (deg/s)');title('Gyro Bias(deg/s)');

figure;
Vertices(:,2) = [gyro_bias_z_value(1:end-1)'*R2D+2*sqrt(gyro_bias_z_error_covariance(1:end-1))'; flipdim(gyro_bias_z_value(1:end-1)'*R2D-2*sqrt(gyro_bias_z_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_z_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('stdv','gyro bias z');grid on;xlabel('time(s)'); ylabel('gyro z bias (deg/s)');title('Gyro Bias(deg/s)');

set(h23,'YDataSource','gyro_bias_x_value(1:end-1)*R2D');
set(h23,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h24,'YDataSource','gyro_bias_y_value(1:end-1)*R2D');
set(h24,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h25,'YDataSource','gyro_bias_z_value(1:end-1)*R2D');
set(h25,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h30 = patch('Faces',faces,'Vertices',CubePoints,'FaceVertexCData',hsv(6),'FaceColor','flat');
%grid on;
axis equal;
view(10,10);
set(h30,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
axis([-X X -X X -X X]);grid on;

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), mag_true_north_heading*R2D, 'b');grid on;title('mag heading (deg)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), roll_quasi(1:end-1)*R2D, 'b');grid on;title('acc roll (deg)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), pitch_quasi(1:end-1)*R2D, 'b');grid on;title('acc pitch (deg)');


figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,13)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('time(s)');hold on;
title('Ground-truth Heading');ylabel('Heading(deg)');

figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,11)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('time(s)');
title('Ground-truth Roll');ylabel('Roll(deg)');

figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,12)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('time(s)');ylabel('Pitch(deg)');
title('Ground-truth Pitch');

figure;
plot(time_vector_sec(1:10:end)-time_vector_sec (1), mag_true_north_heading(1:10:end)*R2D, 'b');grid on;title('mag heading (deg)');
hold on;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,13)*R2D, '-','color','r','linewidth',1);grid on;
xlabel('time(s)');ylabel('Heading(deg)');hold on;
legend('MAG-estimated heading','Ground Truth');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), pitch_quasi(1:end-1)*R2D, 'b');grid on;title('Acc-estimated pitch (deg)');hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), Euler_pitch_value(1:end-1)*R2D, '-','color','r','linewidth',1);grid on;
xlabel('time(s)');ylabel('Pitchdeg)');hold on;
legend('ACC-estimated pitch','EKF');

roll_error = (raw_imu_data(1:end-1,11)-Euler_roll_value(1:end-1)')*R2D;
pitch_error = (raw_imu_data(1:end-1,12)-Euler_pitch_value(1:end-1)')*R2D;
heading_error = (raw_imu_data(1:end-1,13)-Euler_heading_value(1:end-1)')*R2D;
time_vector = time_vector_sec(1:end)-time_vector_sec (1);

q0_error = q_ref(1:end-1,1)-a_value(1:end-1)';
q1_error = q_ref(1:end-1,2)-b_value(1:end-1)';
q2_error = q_ref(1:end-1,3)-c_value(1:end-1)';
q3_error = q_ref(1:end-1,4)-d_value(1:end-1)';

figure;
Vertices(:,2) = [q0_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q0_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q0_error','-.','color','red','linewidth',2);
title('quaternion error:a component');xlabel('time(s)');ylabel('quaternion error:a');
legend('Standard Deviation','quaternion error:(a)');grid on;

figure;
Vertices(:,2) = [q1_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q1_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q1_error','-.','color','red','linewidth',1);grid on;
title('quaternion error:b component');xlabel('time(s)');ylabel('quaternion error:b');
legend('Standard Deviation','quaternion error:(b)');

figure;
Vertices(:,2) = [q2_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q2_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q2_error','-.','color','red','linewidth',2);grid on;
title('quaternion error:c component');xlabel('time(s)');ylabel('quaternion error:c');
legend('Standard Deviation','quaternion error:(c)');

figure;
Vertices(:,2) = [q3_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q3_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q3_error','-.','color','red','linewidth',2);grid on;
title('quaternion error:d component');xlabel('time(s)');ylabel('quaternion error:d');
legend('Standard Deviation','quaternion error:(d)');


figure;plot(time_vector,roll_error);title('Simulation Test: Nominal Design Point: roll error(deg)');xlabel('time(s)');ylabel('roll error(deg)');grid on;
figure;plot(time_vector,pitch_error);title('Simulation Test: Nominal Design Point: pitch error(deg)');xlabel('time(s)');ylabel('pitch error(deg)');grid on;
figure;plot(time_vector,heading_error);title('Simulation Test: Nominal Design Point: heading error(deg)');xlabel('time(s)');ylabel('heading error(deg)');grid on;

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_x_value(1:end-1)), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_y_value(1:end-1)), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_z_value(1:end-1)), '--k');grid on;hold on;
legend('gyro sf x','gyro sf y','gyro sf z');xlabel('time(s)');ylabel('Scale Factor');title('Gyro Scale Factor');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_x_value(1:end-1), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_y_value(1:end-1), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_z_value(1:end-1), '--k');grid on;hold on;
legend('acc bias x','acc bias y','acc bias z');xlabel('time(s)');ylabel('Acc bias(m/s^2)');title('Acc Bias (m/s^2)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_x_value(1:end-1)), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_y_value(1:end-1)), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_z_value(1:end-1)), '--k');grid on;hold on;
legend('acc sf x','acc sf y','acc sf z');xlabel('time(s)');ylabel('Scale Factor');title('Acc Scale Factor');
