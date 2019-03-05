#include "NodeDataManager.h"




NodeDataManager::NodeDataManager( const ros::NodeHandle& _nh ): nh(_nh)
{
node_pose.reserve(10000);
node_timestamps.reserve(10000);
node_pose_covariance.reserve(10000);

loopclosure_edges.reserve(10000);
loopclosure_edges_goodness.reserve(10000);
loopclosure_p_T_c.reserve(10000);

current_kidnap_status = false;
worlds_handle_raw_ptr = new Worlds();

}



void NodeDataManager::camera_pose_callback( const nav_msgs::Odometry::ConstPtr& msg )
{
    // ROS_INFO( "NodeDataManager::camera_pose_callback");


    // timestamp
    //ensure that the node timestamp is newer than the last added node.
    assert( ( node_timestamps.size()==0 )?true:(msg->header.stamp - node_timestamps[ node_timestamps.size()-1 ] )>ros::Duration(0.)   && "I expect the node timestamps to be in order."   );

    // pose
    double orient_w,orient_x,orient_y,orient_z;
    orient_w = msg->pose.pose.orientation.w;
    orient_x = msg->pose.pose.orientation.x;
    orient_y = msg->pose.pose.orientation.y;
    orient_z = msg->pose.pose.orientation.z;
    Quaterniond quat( orient_w,orient_x,orient_y,orient_z );
    Matrix4d w_T_cam = Matrix4d::Zero();
    w_T_cam.topLeftCorner<3,3>() = quat.toRotationMatrix();
    w_T_cam(0,3) = msg->pose.pose.position.x;
    w_T_cam(1,3) = msg->pose.pose.position.y;
    w_T_cam(2,3) = msg->pose.pose.position.z;
    w_T_cam(3,3) = 1.0;
    // cout << "camera_pose_callback: " << node_pose.size() << " " << PoseManipUtils::prettyprintMatrix4d( w_T_cam ) << endl;


    // co-variance
    Matrix<double, 6,6 > Cov;
    for( int i=0 ; i<6 ; i++ )
    {
        for(int j=0 ; j<6 ; j++ )
        {
            Cov(i,j) = msg->pose.covariance[ 6*i + j ];
        }
    }

    // release thread-lock
    // Need thread-lock
    {
        std::lock_guard<std::mutex> lk(node_mutex);
        node_timestamps.push_back( msg->header.stamp );
        node_pose.push_back( w_T_cam );
        node_pose_covariance.push_back( Cov );

        // signal world0 start
        if( node_pose.size() == 1 ) {
            worlds_handle_raw_ptr->world_starts( msg->header.stamp );
        }

    }

}


// void NodeDataManager::loopclosure_pose_callback( const nap::NapMsg::ConstPtr& msg  )
// {
//     // ROS_INFO( "NodeDataManager::loopclosure_pose_callback");
//     // Add a new edge ( 2 node*)
//
//
//     ros::Time t_c = msg->c_timestamp;
//     ros::Time t_p = msg->prev_timestamp;
//     int op_mode = msg->op_mode;
//     double goodness = (double)msg->goodness;
//     assert( op_mode == 30 );
//
//     // retrive rel-pose  p_T_c
//     Matrix4d p_T_c = Matrix4d::Zero();
//     Quaterniond quat( msg->p_T_c.orientation.w, msg->p_T_c.orientation.x, msg->p_T_c.orientation.y, msg->p_T_c.orientation.z );
//     p_T_c.topLeftCorner<3,3>() = quat.toRotationMatrix();
//     p_T_c(0,3) = msg->p_T_c.position.x;
//     p_T_c(1,3) = msg->p_T_c.position.y;
//     p_T_c(2,3) = msg->p_T_c.position.z;
//     p_T_c(3,3) = 1.0;
//
//
//     // Lock
//     node_mutex.lock() ;
//
//     // loop up t_c in node_timestamps[]
//     int index_t_c = find_indexof_node(node_timestamps, t_c );
//
//     // loop up t_p in node_timestamps
//     int index_t_p = find_indexof_node(node_timestamps, t_p );
//
//     // Unlock
//     node_mutex.unlock();
//
//     cout << "[NodeDataManager] Rcvd NapMsg " << index_t_c << "<--->" << index_t_p << endl;
//     assert( t_c > t_p );
//
//     std::pair<int,int> closure_edge;
//     closure_edge.first = index_t_p;
//     closure_edge.second = index_t_c;
//
//     edge_mutex.lock();
//     loopclosure_edges.push_back( closure_edge );
//     loopclosure_edges_goodness.push_back( goodness );
//     loopclosure_p_T_c.push_back( p_T_c );
//     edge_mutex.unlock();
//
//
//
// }

void NodeDataManager::loopclosure_pose_callback(  const cerebro::LoopEdge::ConstPtr& msg  )
{
    // Add a new edge ( 2 node*)


    ros::Time t_a = msg->timestamp0;
    ros::Time t_b = msg->timestamp1;
    // int op_mode = msg->op_mode;
    double goodness = (double)msg->weight;
    // assert( op_mode == 30 );
    string description = msg->description;

    #if 0
    cout <<  TermColor::iYELLOW() << "[NodeDataManager::loopclosure_pose_callback]";
    cout << "t_a=" << t_a << "   t_b=" << t_b << "   wt=" << msg->weight;
    cout << "  description=" << msg->description << TermColor::RESET() << endl;
    #endif


    // retrive rel-pose  p_T_c
    Matrix4d b_T_a = Matrix4d::Zero();
    Quaterniond quat( msg->pose_1T0.orientation.w,
        msg->pose_1T0.orientation.x,
        msg->pose_1T0.orientation.y, msg->pose_1T0.orientation.z );
    b_T_a.topLeftCorner<3,3>() = quat.toRotationMatrix();
    b_T_a(0,3) = msg->pose_1T0.position.x;
    b_T_a(1,3) = msg->pose_1T0.position.y;
    b_T_a(2,3) = msg->pose_1T0.position.z;
    b_T_a(3,3) = 1.0;


    #if 0
    //old code remove this TODO
    // Lock
    node_mutex.lock() ;

    // loop up t_c in node_timestamps[]
    int index_t_a = find_indexof_node(node_timestamps, t_a );

    // loop up t_p in node_timestamps
    int index_t_b = find_indexof_node(node_timestamps, t_b );

    // Unlock
    node_mutex.unlock();
    #endif


    int index_t_a, index_t_b;
    {
        std::lock_guard<std::mutex> lk(node_mutex);
        index_t_a = find_indexof_node(node_timestamps, t_a );
        index_t_b = find_indexof_node(node_timestamps, t_b );

    }

    // cout << "[NodeDataManager] Rcvd Loop Edge " << index_t_c << "<--->" << index_t_p << endl;
    // assert( t_c > t_p );

    std::pair<int,int> closure_edge;
    closure_edge.first = index_t_a;
    closure_edge.second = index_t_b;

    if( index_t_a>=0 && index_t_b >=0   )
    {
        std::lock_guard<std::mutex> lk(edge_mutex);
        loopclosure_edges.push_back( closure_edge );
        loopclosure_edges_goodness.push_back( goodness );
        loopclosure_p_T_c.push_back( b_T_a );
        loopclosure_description.push_back( description );
        loopclosure_edges_timestamps.push_back( std::make_pair( t_a, t_b) );
    }
    else {
        cout << TermColor::YELLOW() << "[NodeDataManager::loopclosure_pose_callback] This edge's end points cannot be found in vector of nodes. This is not FATAL, I am ignoring this edge candidate as a fix.Ideally this should not be happening.\n" << TermColor::RESET() << endl;
    }



}

// internal node queue length info
void NodeDataManager::print_nodes_lengths() const
{
    {
        std::lock_guard<std::mutex> lk(node_mutex);
        cout << "Nodes: pose,cov,stamps=" << node_pose.size() << "," << node_timestamps.size() << "," << node_pose_covariance.size() ;
    }

    {
        std::lock_guard<std::mutex> lk(edge_mutex);
        cout << "\tEdges: pair,weights,edge_pose,description_string=" << loopclosure_edges.size() << "," << loopclosure_edges_goodness.size() << "," << loopclosure_p_T_c.size() << "," << loopclosure_description.size();
    }
    cout << endl;
}

/////////////////////// Utility
// Loop over each node and return the index of the node which is clossest to the specified stamp
// This function is not thread-safe. You need to assure thread safety yourself for this.
int NodeDataManager::find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp )
{
  ros::Duration diff;
  // cout << "find stamp=" << std::setprecision(20) << stamp.toSec();
  // cout << "\t|global_nodes_stamps|="<< global_nodes_stamps.size() ;
  // cout << endl;

  int to_return = -1;
  for( int i=0 ; i<global_nodes_stamps.size() ; i++ ) // TODO how about try backward loop.
  {
    diff = global_nodes_stamps[i] - stamp;

    // cout << i << " "<< diff.sec << " " << diff.nsec << endl;

    if( (diff.sec == 0  &&  abs(diff.nsec) < 1000000) || (diff.sec == -1  &&  diff.nsec > (1000000000-1000000) )  ) {
        // cout << "NodeDataManager::find_indexof_node " << i << " "<< diff.sec << " " << diff.nsec << endl;
        return i;
        to_return = i;
    }


  }

  // cout << "returned idx=" << to_return << endl;
  return to_return;
}


////////////////////////// optimize() ///////////////////////////
// intended to be run in a separate thread.


////////////////// Public interfaces for data... thread safe
int NodeDataManager::getNodeLen() const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    // node_mutex.lock();
    int n = node_pose.size();
    // node_mutex.unlock();
    return n;
}

int NodeDataManager::getEdgeLen() const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    // edge_mutex.lock();
    int n = loopclosure_edges.size();
    // edge_mutex.unlock();
    return n;
}

// returns w_T_cam
bool NodeDataManager::getNodePose( int i, Matrix4d& w_T_cam ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    bool status;
    // node_mutex.lock(); //since this is readonly dont need a lock
    if( i>=0 && i< node_pose.size() )
    {
        w_T_cam = node_pose[i];
        status = true;
    }
    else
    {
        status = false;
    }
    // node_mutex.unlock();

    return status;
}

const Matrix4d& NodeDataManager::getNodePose( int i ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);

    assert( ( i>=0 && i< node_pose.size() ) );
    return node_pose[i];
}


bool NodeDataManager::nodePoseExists( int i) const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    if( ( i>=0 && i< node_pose.size() ) )
        return true;
    return false;

}

bool NodeDataManager::getNodeCov( int i, Matrix<double,6,6>& cov ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);

    bool status;
    // node_mutex.lock(); Since this is readonly dont need a lock
    if( i>=0 && i< node_pose_covariance.size() )
    {
        cov = node_pose_covariance[i];
        status = true;
    }
    else
    {
        status = false;
    }
    // node_mutex.unlock();

    return status;
}

const Matrix<double,6,6>& NodeDataManager::getNodeCov( int i ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    assert(  i>=0 && i< node_pose_covariance.size() );
    return node_pose_covariance[i];
}

bool NodeDataManager::getNodeTimestamp( int i, ros::Time& stamp ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    bool status;
    // node_mutex.lock(); Since this is readonly dont need a lock
    if( i>=0 && i< node_timestamps.size() )
    {
        stamp = node_timestamps[i];
        status = true;
    }
    else
    {
        status = false;
    }
    // node_mutex.unlock();

    return status;
}

const ros::Time NodeDataManager::getNodeTimestamp( int i ) const
{
    std::lock_guard<std::mutex> lk(node_mutex);
    assert( i>=0 && i< node_timestamps.size() );
    return node_timestamps[i];
}

// return p_T_c
bool NodeDataManager::getEdgePose( int i, Matrix4d& p_T_c ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    if( i>=0 && i<loopclosure_edges.size() )
    {
        p_T_c = loopclosure_p_T_c[i];
        return true;
    }
    return false;
}

const Matrix4d& NodeDataManager::getEdgePose( int i ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    assert( i>=0 && i<loopclosure_edges.size() );
    return loopclosure_p_T_c[i];
}

// edge idx info
bool NodeDataManager::getEdgeIdxInfo( int i, std::pair<int,int>& p ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    bool status;
    if( i>=0 && i<loopclosure_edges.size() ) {
        p = loopclosure_edges[i];
        return true;
    }
    return false;
}

const std::pair<int,int>& NodeDataManager::getEdgeIdxInfo( int i ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    assert( i>=0 && i<loopclosure_edges.size() );
    return loopclosure_edges[i];
}

double NodeDataManager::getEdgeWeight( int i ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    if( i>=0 && i<loopclosure_edges.size() ) {
        return loopclosure_edges_goodness[i];
    }
    return -1.0;
}


const string NodeDataManager::getEdgeDescriptionString( int i ) const
{
    std::lock_guard<std::mutex> lk(edge_mutex);
    if( i>=0 && i<loopclosure_edges.size() ) {
        return loopclosure_description[i];
    }
    return "NA";


}





void NodeDataManager::reset_edge_info_data()
{
    std::lock_guard<std::mutex> lk(edge_mutex);

    loopclosure_edges.clear();
    loopclosure_edges_goodness.clear();
    loopclosure_p_T_c.clear();
    loopclosure_description.clear();
}

void NodeDataManager::reset_node_info_data()
{
    std::lock_guard<std::mutex> lk(node_mutex);

    node_pose.clear();
    node_timestamps.clear();
    node_pose_covariance.clear();
}


bool NodeDataManager::saveAsJSON( const string& base_path )
{
    json all_info;
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ",", ";");


    all_info["meta_data"]["getNodeLen"] = getNodeLen();
    all_info["meta_data"]["getEdgeLen"] = getEdgeLen();

    // Node Info
    for( int i=0 ; i<getNodeLen() ; i++ )
    {
        // node_mutex.lock();
        // ros::Time stamp;
        // getNodeTimestamp( i, stamp );

        json node;
        node["timestamp"] = getNodeTimestamp(i).toSec();
        node["idx"] = i;

        Matrix4d wTc;
        getNodePose(i, wTc );
        std::stringstream ss;
        ss << wTc.format(CSVFormat);
        node["wTc"] = ss.str();

        Matrix<double,6,6> cov;
        std::stringstream ss2;
        ss2 << cov.format(CSVFormat);
        getNodeCov( i, cov );
        node["cov"] = ss2.str();

        all_info["nodes"].push_back( node );

        // node_mutex.unlock();

    }



    // Edge Info
    for( int i=0; i<getEdgeLen() ; i++ )
    {
        // edge_mutex.lock();
        json edge;

        const std::pair<int,int> p = getEdgeIdxInfo(i);

        edge["idx0"] = p.first;
        edge["idx1"] = p.second;
        edge["timestamp0"] = getNodeTimestamp(p.first).toSec();
        edge["timestamp1"] = getNodeTimestamp(p.second).toSec();


        const Matrix4d b_T_a = getEdgePose(i);
        std::stringstream ss;
        ss << b_T_a.format(CSVFormat);
        edge["b_T_a"] = ss.str();

        edge["weight"] = getEdgeWeight( i );
        edge["description"] = getEdgeDescriptionString(i);
        // edge_mutex.unlock();

        all_info["loopedges"].push_back( edge );


    }


    // Save file
    cout << TermColor::GREEN() << "[NodeDataManager::saveAsJSON]\n" <<  all_info["meta_data"] << endl;
    cout << "Write : "<< base_path+"/log_posegraph.json"<< endl;
    cout << "Done! "<< TermColor::RESET() << endl;
    std::ofstream outf(base_path+"/log_posegraph.json");
    if( !outf.is_open() ) {
        cout << "[ERROR saveAsJSON] Cannot open file\n";
        return false;
    }

    outf << std::setw(4) << all_info << std::endl;
    return true;


}


bool NodeDataManager::loadFromJSON( const string& base_path, const vector<bool>& edge_mask ) //< Loads what saveForDebug() writes. edge_mask: a vector of 0s, 1s indicating if this edge has to be included or not. edge_mask.size() == 0 will load all
{
    cout << "##########################################\n";
    cout << " NodeDataManager::loadFromJSON : "<< base_path << endl;
    cout << "##########################################\n";

    cout << "edge_mask.size()="<< edge_mask.size() << endl;
    for( int i=0 ; i<edge_mask.size() ; i++ )
        cout << edge_mask[i];
    cout << endl;

    reset_node_info_data();
    reset_edge_info_data();


    // Open JSON file
    cout << TermColor::GREEN() << "Open JSON : "<< base_path+"/log_posegraph.json"<< TermColor::RESET() << endl;
    std::ifstream i_file(base_path+"/log_posegraph.json");
    if( !i_file.is_open() ) {
        cout << TermColor::RED() << "[ERROR loadFromJSON]Cannot open json file. Perhaps file not found" << TermColor::RESET() << endl;
        return false;
    }
    json all_info;
    i_file >> all_info;

    cout << all_info["meta_data"] << endl;
    int getEdgeLen = all_info["meta_data"]["getEdgeLen"];
    int getNodeLen = all_info["meta_data"]["getNodeLen"];
    if( getEdgeLen != all_info["loopedges"].size() || getNodeLen != all_info["nodes"].size() )
    {
        cout << "The meta data and the json file is not consistant\n";
        cout << "meta[getEdgeLen]" << getEdgeLen << "\tmeta[getNodeLen]" << getNodeLen ;
        cout << "all_info[\"loopedges\"].size()" << all_info["loopedges"].size() ;
        cout << "\tall_info[\"nodes\"].size()" << all_info["nodes"].size() ;
        return false;
    }

    // Load Nodes
    for( int i=0 ; i<all_info["nodes"].size() ; i++ )
    {
        json node_json = all_info["nodes"][i];
        // cout << node_json << endl;

        // lock
        std::lock_guard<std::mutex> lk(node_mutex);


        // timestamp
        ros::Time node_time( all_info["nodes"][i]["timestamp"] );
        node_timestamps.push_back( node_time );

        // pose
        Matrix4d wTc;
        PoseManipUtils::string_to_eigenmat( all_info["nodes"][i]["wTc"], wTc );
        node_pose.push_back( wTc );
        // cout << i<< "\n" << wTc << endl;


        // cov
        Matrix<double,6,6> cov;
        PoseManipUtils::string_to_eigenmat( node_json["cov"], cov );
        node_pose_covariance.push_back( cov );
    }
    cout << TermColor::GREEN() << "Loaded Nodes: this->getNodeLen()=" << this->getNodeLen() << TermColor::RESET() << endl;


    // Load Edges
    for( int i=0 ; i<all_info["loopedges"].size() ; i++ )
    {
        if( edge_mask.size() > 0 && edge_mask[i] == false )
            continue;// dont load the edge if edge_mask has non zero size and mask is true.


        // cout << "---\n";
        json loopedge_json = all_info["loopedges"][i];
        // cout << loopedge_json << endl;

        std::lock_guard<std::mutex> lk(edge_mutex);

        // load stamp0, stamp1
        ros::Time stamp0( loopedge_json["timestamp0"] );
        ros::Time stamp1( loopedge_json["timestamp1"] );
        loopclosure_edges_timestamps.push_back( std::make_pair(stamp0,stamp1) );
        // cout << stamp0 << "\t" << stamp1 << endl;

        // load id
        int idx0 = loopedge_json["idx0"];
        int idx1 = loopedge_json["idx1"];
        loopclosure_edges.push_back( make_pair(idx0, idx1) );
        // cout << idx0 << "\t" << idx1 << endl;

        // load weight & description
        loopclosure_edges_goodness.push_back( loopedge_json["weight"] );
        loopclosure_description.push_back( loopedge_json["description"] );
        // cout << loopedge_json["weight"] << "\t" << loopedge_json["description"] << endl;


        // load relative pose
        Matrix4d b_T_a;
        PoseManipUtils::string_to_eigenmat( loopedge_json["b_T_a"], b_T_a );
        loopclosure_p_T_c.push_back( b_T_a );
        // cout << i<< " b_T_a\n" << b_T_a << endl;


        // verify that these timestamp exists in node_timestamps and are equal to claimed
        if( node_timestamps[idx0] != stamp0 ) {
            cout << "[Insonsistent json] node_timestamps[idx0] != stamp0 \n";
            cout << "node_timestamps[idx0]=" << node_timestamps[idx0] << endl;
            cout << "stamp0=" << stamp0 << endl;
            exit(1);
        }
        if( node_timestamps[idx1] != stamp1 ) {
            cout << "[Insonsistent json] node_timestamps[idx1] != stamp1 \n";
            cout << "node_timestamps[idx1]=" << node_timestamps[idx1] << endl;
            cout << "stamp1=" << stamp1 << endl;
            exit(1);
        }


    }
    cout << TermColor::GREEN() << "Loaded LoopEdges: this->getEdgeLen()=" << this->getEdgeLen() << TermColor::RESET() << endl;


}


///// kidnap related

// The header contains a timestamp. indicator string is frame_id.
// frame_id == "kidnapped" ==> the timestamp is the time of kidnap
// frame_id == "unkidnapped" ==> the timestamp is the time of unkidnapped.
//  anything else in frame_id is an error
void NodeDataManager::rcvd_kidnap_indicator_callback( const std_msgs::HeaderConstPtr& rcvd_header )
{
    cout << TermColor::RED() << "[posegraph solver rcvd_kidnap_indicator_callback]" ;
    cout << rcvd_header->stamp << ":" << rcvd_header->frame_id ;
    cout << TermColor::RESET() << endl;

    if( rcvd_header->frame_id  == "kidnapped" ) {
        // the time stamp is the start of kidnap
        mark_as_kidnapped( rcvd_header->stamp );

        // signal the world ended
        worlds_handle_raw_ptr->world_ends( rcvd_header->stamp );
        return;


    }

    if( rcvd_header->frame_id  == "unkidnapped" ) {
        // the timestamp is end of kidnap
        mark_as_unkidnapped( rcvd_header->stamp );

        // signal start of a new world
        worlds_handle_raw_ptr->world_starts( rcvd_header->stamp );
        return;
    }

    ROS_ERROR( "[posegraph solver NodeDataManager::rcvd_kidnap_indicator_callback] rcvd_header is something other than `kidnapped` or `unkidnapped`. This should not be happening and is a fatal error.");
}


void NodeDataManager::mark_as_kidnapped( const ros::Time _t )
{
    assert( current_kidnap_status == false && "[NodeDataManager::mark_as_kidnapped] you can mark as kidnapped only when i was not kidnapped.\n");
    {
        std::lock_guard<std::mutex> lk(mutex_kidnap);
        current_kidnap_status = true;
        kidnap_starts.push_back( _t );
    }
}

void NodeDataManager::mark_as_unkidnapped( const ros::Time _t )
{
    assert( current_kidnap_status == true && "[NodeDataManager::mark_as_unkidnapped] you can mark as unkidnapped only when i was kidnapped.\n");
    {
        std::lock_guard<std::mutex> lk(mutex_kidnap);
        current_kidnap_status = false;
        kidnap_ends.push_back( _t );
    }
}

const ros::Time NodeDataManager::last_kidnap_ended() const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    if( kidnap_ends.size() > 0 ) {
        return kidnap_ends[ kidnap_ends.size()-1 ];
    }
    else {
        return ros::Time();
    }
}

const ros::Time NodeDataManager::last_kidnap_started() const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    if( kidnap_starts.size() > 0 ) {
        return kidnap_starts[ kidnap_starts.size()-1 ];
    }
    else {
        return ros::Time();
    }
}


int NodeDataManager::n_kidnaps() const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    return kidnap_ends.size();
}

// #define __KIDNAP_START_ENDS___debug( msg ) msg;
#define __KIDNAP_START_ENDS___debug( msg ) ;
ros::Time NodeDataManager::stamp_of_kidnap_i_started( int i ) const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);

    if( i>=0 && i<kidnap_starts.size() ) {
        return kidnap_starts[i];
    }

    __KIDNAP_START_ENDS___debug( cout << "[NodeDataManager::stamp_of_kidnap_i_started]no such kidnap" << i << " kidnap_starts.size()=" << kidnap_starts.size() << endl; )
    return ros::Time();
}

ros::Time NodeDataManager::stamp_of_kidnap_i_ended( int i ) const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    if( i>=0 && i<kidnap_ends.size() ) {
        return kidnap_ends[i];
    }

    __KIDNAP_START_ENDS___debug( cout << "[NodeDataManager::stamp_of_kidnap_i_ended]no such kidnap" << i << " kidnap_ends.size()=" << kidnap_ends.size() << endl; )
    return ros::Time();
}

int NodeDataManager::which_world_is_this( const ros::Time _t ) const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);


    if( kidnap_starts.size() == 0 ) // there are no kidnaps, so always return '0'
        return 0;

    if( kidnap_starts.size() == 1 ) {
        if( _t < kidnap_starts[0] )
            return 0;

        if( kidnap_ends.size() == 0 ) {
            if( _t >= kidnap_starts[0] )
                return -1;
            else
                return 0;
        } else {
            if( _t >= kidnap_starts[0] && _t <= kidnap_ends[0] )
                return -1;
            else
                return 1;
        }
    }

    // ROS_ERROR( "NOT implemented [int which_world_is_this( const ros::Time _t )]. EXITE10");
    // exit(10);



    if( kidnap_starts.size() == kidnap_ends.size() )
    {
        ros::Time prev = ros::Time(); // TODO deally should, set this to pose0timestamp
        for( int i=0 ; i<kidnap_starts.size() ; i++ )
        {
            if( _t > prev && _t <= kidnap_starts[i] )
                return i;

            if( _t> kidnap_starts[i] && _t <= kidnap_ends[i] )
                return -(i+1);

            prev = kidnap_ends[i];
        }
        return kidnap_ends.size();
    } else {

        // this means the current state is kidnapped.
        ros::Time prev = ros::Time(); // TODO deally should, set this to pose0timestamp
        for( int i=0 ; i<kidnap_starts.size()-1 ; i++ )
        {
            if( _t > prev && _t <= kidnap_starts[i] )
                return i;

            if( _t> kidnap_starts[i] && _t <= kidnap_ends[i] )
                return -(i+1);

            prev = kidnap_ends[i];
        }

        int i = kidnap_starts.size() - 1;
        if( _t > kidnap_ends[i-1] && _t <= kidnap_starts[i] )
            return i;

        if( _t > kidnap_starts[i] )
            return -(i+1);




    }

}

/*
int NodeDataManager::which_world_is_this( int i ) //given the node idx, gets the which_world_is_this.
{
    const ros::Time _t = this->getNodeTimestamp( i );
    return which_world_is_this( _t );

}
*/


// #define __WORLD_START_ENDS___debug( msg ) msg;
#define __WORLD_START_ENDS___debug( msg ) ;
int NodeDataManager::nodeidx_of_world_i_started( int i ) const
{

    if( i<0 )
    {
        __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_started] i cant be negative. no such world " << i << " exists\n"; )
        return -3;
    }
    if( i==0 ) {
        __WORLD_START_ENDS___debug(
        cout << "[NodeDataManager::nodeidx_of_world_i_started] special case of world0\n";
        )
        return 0;
    }


    int n=0;
    {
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    n=kidnap_ends.size() ;
    }

    if( i>=1 && (i-1) <n ) {
        __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_started] return nodeidx of kidnap_ends["<<i-1 <<"] as the start of world" << i << "\n"; )
        std::lock_guard<std::mutex> lk(node_mutex);

        int r=0;
        for( auto it=node_timestamps.begin() ; it!=node_timestamps.end() ; it++, r++  ) {
            if( which_world_is_this( *it ) == i )
                return r;
        }

        // return -1;
        // return find_indexof_node( node_timestamps, kidnap_ends[i-1]  );
    }


    __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_started] no such world " << i << " exists\n"; )
    return -4;
}

int NodeDataManager::nodeidx_of_world_i_ended( int i ) const
{
    // returns a large number if the world i never ended
    int n_kidnap_ends;
    {
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    n_kidnap_ends = kidnap_ends.size();
    }

    if( i<0 ) {
        __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_ended] i cannot be negative. no such world\n");
        return -1;
    }

    if( i>n_kidnap_ends ) {
        __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_ended] no such world" << i << "\n"; )
        return -1;
    }
    else {
        // ith world exist
        std::lock_guard<std::mutex> lk(node_mutex);
        __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_ended] world" << i << " exists\n"; )
        if( i>=0 && i<kidnap_starts.size() ) {
            __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_ended] return kidnap_start[" << i << "]\n"; )
            return find_indexof_node( node_timestamps, kidnap_starts[i]  );
        }
        else {
            __WORLD_START_ENDS___debug( cout << "[NodeDataManager::nodeidx_of_world_i_ended] world"<< i << " never ends\n"; )
            return node_timestamps.size()-1;
        }
    }



}





int NodeDataManager::n_worlds() const
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    return kidnap_ends.size() + 1;
}
