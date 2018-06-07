#pragma once
/** class PoseGraphSLAM

        This runs in a separate thread. Retrives data from NodeDataManager
        on the node's poses and closure edges. Does the non-linear
        least squares.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 6th, June, 2018

*/
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>


#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>


// ros
#include <ros/ros.h>
#include <ros/package.h>

#include "NodeDataManager.h"

using namespace std;
using namespace Eigen;

class PoseGraphSLAM
{
public:
    PoseGraphSLAM( NodeDataManager* _manager );
    // void stop() { run = false; }

    // This is intended to be run in a separate thread.
    void optimize6DOF();

private:
    //TODO leran how to correctly use atomic.

    const NodeDataManager * manager;
};



class SixDOFError
{
public:
    SixDOFError( const Matrix4d& observed__i_T_j )
    {

    }

private:
    Matrix4d observed__i_T_j;
}
