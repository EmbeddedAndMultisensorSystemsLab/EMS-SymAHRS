%   normalize_quaternion

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

function [ q1_out, q2_out, q3_out, q4_out ] = normalize_quaternion(q_in)
    norm_value = norm(q_in);
    q1_out  = q_in(1)/norm_value;
    q2_out  = q_in(2)/norm_value;
    q3_out  = q_in(3)/norm_value;
    q4_out  = q_in(4)/norm_value;
end