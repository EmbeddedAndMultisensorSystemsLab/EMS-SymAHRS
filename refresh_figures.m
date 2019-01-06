%   Plots Update

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
global time_vector_sec;
refreshdata(h1);
refreshdata(h2);
refreshdata(h3);
refreshdata(h4);
refreshdata(h5);
refreshdata(h6);

refreshdata(h13);
refreshdata(h14);
refreshdata(h15);
refreshdata(h15);
refreshdata(h16);
refreshdata(h17);
refreshdata(h18);

refreshdata(h23);
refreshdata(h24);
refreshdata(h25);

set(h30,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);

delay(0.03);