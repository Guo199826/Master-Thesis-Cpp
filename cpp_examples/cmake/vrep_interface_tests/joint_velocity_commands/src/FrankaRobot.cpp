/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#ifndef DQ_ROBOTICS_FRANKA_DH_H
#define DQ_ROBOTICS_FRANKA_DH_H

#include "../include/FrankaRobot.h"
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

    DQ_SerialManipulatorDH FrankaRobot::kinematics()
    {
        const double pi2 = pi/2.0;

        MatrixXd franka_dh(5,7);
        franka_dh <<    0,      0,          0,          0,      0,      0,          0,
                    0.333,      0,      0.316,          0,  0.384,      0,      0.107,
                        0,      0,     0.0825,    -0.0825,      0,  0.088,     0.0003,
                    -pi2,    pi2,        pi2,       -pi2,    pi2,    pi2,          0,
                        0,      0,          0,          0,      0,      0,          0;
        DQ_SerialManipulatorDH franka(franka_dh);
        std::cout<<"kinematics running..."<<std::endl;
        return franka;
    }

    MatrixXd _get_mdh_matrix()
    {
        const double pi2 = pi/2.0;
        Matrix<double,5,7> raw_franka_mdh(5,7);
        raw_franka_mdh <<  0,    0,       0,         0,         0,      0,      0,
                        0.333,  0, 3.16e-1,         0,   3.84e-1,      0,      0,
                        0,     0,       0,   8.25e-2,  -8.25e-2,      0, 8.8e-2,
                        0,  -pi2,     pi2,       pi2,      -pi2,    pi2,    pi2,
                        0,     0,       0,         0,         0,      0,      0;

        return raw_franka_mdh;
    }

}

#endif
