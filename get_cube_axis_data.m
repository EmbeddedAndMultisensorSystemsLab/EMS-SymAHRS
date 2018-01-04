%   get_cube_axis_data

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

function [ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data( ver )
CubeXData = [[ver(1,1);ver(2,1);ver(3,1);ver(4,1)] [ver(4,1);ver(3,1);ver(5,1);ver(6,1)]...
    [ver(6,1);ver(7,1);ver(8,1);ver(5,1)] [ver(1,1);ver(2,1);ver(8,1);ver(7,1)]...
    [ver(6,1);ver(7,1);ver(1,1);ver(4,1)] [ver(2,1);ver(3,1);ver(5,1);ver(8,1)]];
CubeYData = [[ver(1,2);ver(2,2);ver(3,2);ver(4,2)] [ver(4,2);ver(3,2);ver(5,2);ver(6,2)]...
    [ver(6,2);ver(7,2);ver(8,2);ver(5,2)] [ver(1,2);ver(2,2);ver(8,2);ver(7,2)]...
    [ver(6,2);ver(7,2);ver(1,2);ver(4,2)] [ver(2,2);ver(3,2);ver(5,2);ver(8,2)]];
CubeZData = [[ver(1,3);ver(2,3);ver(3,3);ver(4,3)] [ver(4,3);ver(3,3);ver(5,3);ver(6,3)]...
    [ver(6,3);ver(7,3);ver(8,3);ver(5,3)] [ver(1,3);ver(2,3);ver(8,3);ver(7,3)]...
    [ver(6,3);ver(7,3);ver(1,3);ver(4,3)] [ver(2,3);ver(3,3);ver(5,3);ver(8,3)]];
end

