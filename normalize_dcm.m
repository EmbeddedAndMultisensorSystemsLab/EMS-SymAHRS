%   normalize_dcm

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

function [ DCM_out ] = normalize_dcm( DCM_in)
    DCM_out(1,:) = DCM_in(1,:)/norm(DCM_in(1,:));
    DCM_out(2,:) = DCM_in(2,:)/norm(DCM_in(2,:));
    DCM_out(3,:) = DCM_in(3,:)/norm(DCM_in(3,:));
end

