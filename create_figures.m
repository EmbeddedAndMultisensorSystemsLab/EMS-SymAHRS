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
h1 = plot(time_vector_sec-time_vector_sec (1), Euler_heading_value(1,1:end-1)*R2D, 'r');
legend('heading');grid on;xlabel('time(sec)'); ylabel('heading(deg)');
set(h1,'YDataSource','Euler_heading_value(1,1:end-1)*R2D');
set(h1,'XDataSource','time_vector_sec-time_vector_sec (1)');
figure;
h3 = plot(time_vector_sec-time_vector_sec (1), Euler_roll_value(1,1:end-1)*R2D, 'b');hold on;
h2 = plot(time_vector_sec-time_vector_sec (1), Euler_pitch_value(1,1:end-1)*R2D, 'r');
legend('roll','pitch');grid on;xlabel('time(sec)'); ylabel('roll/pitch(deg)');
set(h2,'YDataSource','Euler_pitch_value(1,1:end-1)*R2D');
set(h2,'XDataSource','time_vector_sec-time_vector_sec (1)');
set(h3,'YDataSource','Euler_roll_value(1,1:end-1)*R2D');
set(h3,'XDataSource','time_vector_sec-time_vector_sec (1)');

figure;
h4 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_x*R2D, 'b');hold on;
h5 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_y*R2D, 'g');
h6 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_z*R2D, 'r');
legend('gyro x','gyro y','gyro z');grid on;xlabel('time(sec)'); ylabel('raw gyro(deg/s)');
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
legend('acc x','acc y','acc z');grid on;xlabel('time(sec)'); ylabel('raw acc(m/s2)');
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
legend('mag x','mag y','mag z');grid on;xlabel('time(sec)'); ylabel('mag (uT)');
set(h16,'YDataSource','raw_mag_x');
set(h16,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h17,'YDataSource','raw_mag_y');
set(h17,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h18,'YDataSource','raw_mag_z');
set(h18,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h23 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_x_value(1:end-1)*R2D, 'r');grid on;hold on;
h24 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_y_value(1:end-1)*R2D, 'g');grid on;hold on;
h25 = plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_z_value(1:end-1)*R2D, 'b');grid on;hold on;
legend('gyr bias x','gyr bias y', 'gyr bias z');grid on;xlabel('time(sec)'); ylabel('gyro bias (deg/s)');
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

