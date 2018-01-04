%   get_euler_from_dcm

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

function [ roll,pitch, heading ] = get_euler_from_dcm( DCM_in)
    pitch     = atan2(-DCM_in(3,1),(sqrt(DCM_in(3,2)^2 + DCM_in(3,3)^2)));
    roll      = atan2(DCM_in(3,2),DCM_in(3,3));
    heading   = atan2(DCM_in(2,1),DCM_in(1,1));
end

