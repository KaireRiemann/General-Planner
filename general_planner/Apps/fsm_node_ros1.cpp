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

#include "ros_interface/ros1/fsm_ros1.hpp"

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "utils/header/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


using namespace fsm;
using namespace std;

#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN << " -- [Fsm-Test] Begin." << RESET << endl;

#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
    std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
    std::string cfg_path, cfg_name;
    const bool has_cfg_path = nh.getParam("config_path", cfg_path) && !cfg_path.empty();
    const bool has_cfg_name = nh.getParam("config_name", cfg_name) && !cfg_name.empty();
    if(has_cfg_name){
        cfg_path = CONFIG_FILE_DIR(cfg_name);
        cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
    } else if (has_cfg_path) {
        cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
    } else {
        cfg_path = dft_cfg_path;
        cout << " -- [Fsm-Test] Load default config: " << cfg_path << endl;
    }

    auto fsm_ptr = make_shared<FsmRos1>();
    fsm_ptr->init(nh, cfg_path);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    spinner.stop();
    fsm_ptr.reset();
    return 0;
}
