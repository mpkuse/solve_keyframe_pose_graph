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

#include "utils/DisjointSet.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std;
using namespace Eigen;


class Worlds
{
public:
    Worlds() {}

    const Matrix4d getPoseBetweenWorlds( int m, int n ) const ;
    bool setPoseBetweenWorlds( int m, int n, const Matrix4d m_T_n, const string info_string );
    bool is_exist( int m, int n ) const ;
    void getAllKeys( vector<std::pair<int,int> >& all_keys ) const ;

    void print_summary() const;
    const string disjoint_set_status() const ;



    // Just infor me on start of the worlds
    void world_starts( ros::Time _t );
    void world_ends( ros::Time _t );


    // Set related (query from disjoiunt set data-structure)
    int find_setID_of_world_i( int i ) const; //< returns the setid of world i
    int n_worlds() const; //< number of worlds
    int n_sets() const;   //< number of sets


private:
    // mutable std::mutex  changes_to_the_world TODO ;
    mutable std::mutex mutex_world;


    std::map< std::pair<int,int> , Matrix4d > rel_pose_between_worlds__wb_T_wa;
    std::map< std::pair<int,int> , string > rel_pose_between_worlds__wb_T_wa___info_string;

    //TODO:  also have start of world and end of worlds timestamps
    vector<ros::Time> vec_world_starts;
    vector<ros::Time> vec_world_ends;

    //disjoint set. TODO: best if you use the boost::disjoint_set rather than a DIY one. the DIY one is prone to errors in implementation.
    mp::DisjointSetForest<bool> disjoint_set; //
    string disjoint_set_debug;


};
