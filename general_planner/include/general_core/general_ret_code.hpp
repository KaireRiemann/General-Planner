/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GENERAL_RET_CODE_HPP
#define GENERAL_RET_CODE_HPP

#include <cstring>
#include <vector>

namespace general_planner {
    enum GENERAL_RET_CODE {
        GENERAL_EXPLORATION_FINISH = 4,
        GENERAL_SUCCESS_WITH_BACKUP = 3,
        GENERAL_SUCCESS_NO_BACKUP = 2,
        GENERAL_SUCCESS = 1,
        GENERAL_UNDEFINED = 0,
        GENERAL_NO_ODOM = -1,
        GENERAL_NO_START_POINT = -2,
        GENERAL_MAP_NOT_READY = -3,

    };

    static std::string GENERAL_RET_CODE_STR(const int& ret) {
        switch (ret) {
        case GENERAL_EXPLORATION_FINISH:
            return "Exploration finished";
        case GENERAL_SUCCESS_WITH_BACKUP:
            return "Success, with backup trajectory also success";
        case GENERAL_SUCCESS_NO_BACKUP:
            return "Success, without need of backup";
        case GENERAL_SUCCESS:
            return "Success";
        case GENERAL_UNDEFINED:
            return "Undefined";
        case GENERAL_NO_ODOM:
            return "No odom, return at the start of the replan";
        case GENERAL_NO_START_POINT:
            return "Cannot find a start point in the local map";
        case GENERAL_MAP_NOT_READY:
            return "Map is not ready";
        }
        return "Unknown return code";
    };
}
#endif
