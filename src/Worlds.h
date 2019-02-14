#pragma once

// This class is used to implement kidnap merging.
// This will hold all the relative poses between 2 worlds. This is a collection
// of all those found merges. May be in the future make use of
// disjoint sets to hold all the relations.

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <map>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include "utils/PoseManipUtils.h"
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"


#include "nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std;
using namespace Eigen;


class Worlds
{
public:
    Worlds() {}

    const Matrix4d getPoseBetweenWorlds( int m, int n ) const ;
    bool setPoseBetweenWorlds( int m, int n, const Matrix4d m_T_n );
    bool is_exist( int m, int n ) const ;
    void getAllKeys( vector<std::pair<int,int> >& all_keys ) const ;

    void print_summary(); 

private:

    std::map< std::pair<int,int> , Matrix4d > rel_pose_between_worlds__wb_T_wa;

    //TODO:  also have start of world and end of worlds timestamps

    //TODO: disjoint set


};
