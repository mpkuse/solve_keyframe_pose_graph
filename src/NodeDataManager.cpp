#include "NodeDataManager.h"




NodeDataManager::NodeDataManager( const ros::NodeHandle& _nh ): nh(_nh)
{
node_pose.reserve(10000);
node_timestamps.reserve(10000);
node_pose_covariance.reserve(10000);

loopclosure_edges.reserve(10000);
loopclosure_edges_goodness.reserve(10000);
loopclosure_p_T_c.reserve(10000);
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
    // ROS_INFO( "NodeDataManager::loopclosure_pose_callback");
    // Add a new edge ( 2 node*)


    ros::Time t_c = msg->timestamp0;
    ros::Time t_p = msg->timestamp1;
    // int op_mode = msg->op_mode;
    double goodness = (double)msg->weight;
    // assert( op_mode == 30 );
    string description = msg->description;

    // retrive rel-pose  p_T_c
    Matrix4d p_T_c = Matrix4d::Zero();
    Quaterniond quat( msg->pose_1T0.orientation.w, msg->pose_1T0.orientation.x, msg->pose_1T0.orientation.y, msg->pose_1T0.orientation.z );
    p_T_c.topLeftCorner<3,3>() = quat.toRotationMatrix();
    p_T_c(0,3) = msg->pose_1T0.position.x;
    p_T_c(1,3) = msg->pose_1T0.position.y;
    p_T_c(2,3) = msg->pose_1T0.position.z;
    p_T_c(3,3) = 1.0;


    // Lock
    node_mutex.lock() ;

    // loop up t_c in node_timestamps[]
    int index_t_c = find_indexof_node(node_timestamps, t_c );

    // loop up t_p in node_timestamps
    int index_t_p = find_indexof_node(node_timestamps, t_p );

    // Unlock
    node_mutex.unlock();

    cout << "[NodeDataManager] Rcvd Loop Edge " << index_t_c << "<--->" << index_t_p << endl;
    assert( t_c > t_p );

    std::pair<int,int> closure_edge;
    closure_edge.first = index_t_p;
    closure_edge.second = index_t_c;


    {
        std::lock_guard<std::mutex> lk(edge_mutex);
        loopclosure_edges.push_back( closure_edge );
        loopclosure_edges_goodness.push_back( goodness );
        loopclosure_p_T_c.push_back( p_T_c );
        loopclosure_description.push_back( description );
    }



}


/////////////////// Publish
void NodeDataManager::setVisualizationPublisher( const ros::Publisher& pub )
{
  // usually was "/mish/pose_nodes"
  pub_pgraph = pub;
}

void NodeDataManager::setPathPublisher( const ros::Publisher& pub )
{
  // usually was "/mish/pose_nodes"
  pub_path_opt = pub;
}



void NodeDataManager::publishLastNNodes( int n )
{
    visualization_msgs::Marker marker ;
    RosMarkerUtils::init_camera_marker( marker, .8 );

    node_mutex.lock();
    int len = node_pose.size();
    node_mutex.unlock();

    int start, end;
    if( n<= 0 )
        start = 0;
    else
        start = max(0,len-n);

    end = len;

    for( int i=start ; i<end ; i++ )
    {
        // node_mutex.lock();
        Matrix4d w_T_c = node_pose[i];
        // node_mutex.unlock();
        // cout << "publish "<< i << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
        RosMarkerUtils::setpose_to_marker( w_T_c, marker );
        RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, marker );
        marker.id = i;
        marker.ns = "vio_kf_pose";

        pub_pgraph.publish( marker );
    }
}

void NodeDataManager::publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b )
{
    // this is a cost effective way to visualize camera path
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x *= 2;
    marker.ns = ns;


    std_msgs::ColorRGBA C1;
    C1.r = r; C1.g=g; C1.b=b; C1.a = 0.8;

    marker.points.clear();
    marker.colors.clear();
    for( int i=0 ; i<w_T_ci.size() ; i++ )
    {
        geometry_msgs::Point pt;
        pt.x = (w_T_ci[i])(0,3);
        pt.y = (w_T_ci[i])(1,3);
        pt.z = (w_T_ci[i])(2,3);

        marker.points.push_back( pt );
        marker.colors.push_back( C1 );
    }

    pub_pgraph.publish( marker );


}


void NodeDataManager::publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1, bool enable_camera_visual  )
{
    assert( idx_partition <= w_T_ci.size() );


    // this is a cost effective way to visualize camera path
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.02;

    std_msgs::ColorRGBA C1;
    C1.r = r; C1.g=g; C1.b=b; C1.a = 0.8;
    std_msgs::ColorRGBA C2;
    C2.r = r1; C2.g=g1; C2.b=b1; C2.a = 0.8;

    marker.points.clear();
    marker.colors.clear();
    marker.id = 0;
    marker.ns = ns;
    for( int i=0 ; i< idx_partition ; i++ )
    {
        geometry_msgs::Point pt;
        pt.x = (w_T_ci[i])(0,3);
        pt.y = (w_T_ci[i])(1,3);
        pt.z = (w_T_ci[i])(2,3);

        marker.points.push_back( pt );
        marker.colors.push_back( C1 );
    }

    for( int i=idx_partition; i<w_T_ci.size() ; i++ )
    {
        geometry_msgs::Point pt;
        pt.x = (w_T_ci[i])(0,3);
        pt.y = (w_T_ci[i])(1,3);
        pt.z = (w_T_ci[i])(2,3);

        marker.points.push_back( pt );
        marker.colors.push_back( C2 );
    }

    pub_pgraph.publish( marker );

    if( enable_camera_visual )
    {
        int last_idx = w_T_ci.size() - 1;
        visualization_msgs::Marker marker2 ;
        RosMarkerUtils::init_camera_marker( marker2, 10 );
        RosMarkerUtils::setpose_to_marker( w_T_ci[last_idx], marker2 );
        RosMarkerUtils::setcolor_to_marker( r,g,b, marker2 );
        marker2.scale.x = 0.02;
        marker2.id = 0;
        marker2.ns = ns+string("_cam_visual");
        pub_pgraph.publish( marker2 );
    }

}



void NodeDataManager::publishEdgesAsLineArray( int n )
{
    int start, end;
    edge_mutex.lock();
    int len = loopclosure_edges.size();
    edge_mutex.unlock();

    if( n<= 0 )
        start = 0;
    else
        start = max( 0, len-n );

    end = len;

    // make a line marker
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );

    for( int i=start; i<end ; i++ )
    {
        edge_mutex.lock();
        int idx_node_prev = loopclosure_edges[i].first ;
        int idx_node_curr = loopclosure_edges[i].second;
        edge_mutex.unlock();

        node_mutex.lock();
        Vector3d w_t_prev = node_pose[ idx_node_prev ].col(3).head(3);
        Vector3d w_t_curr = node_pose[ idx_node_curr ].col(3).head(3);
        node_mutex.unlock();


        geometry_msgs::Point pt;
        pt.x = w_t_prev(0);
        pt.y = w_t_prev(1);
        pt.z = w_t_prev(2);
        marker.points.push_back( pt );
        pt.x = w_t_curr(0);
        pt.y = w_t_curr(1);
        pt.z = w_t_curr(2);
        marker.points.push_back( pt );

    }
    marker.ns = "loop_closure_edges_ary";
    marker.id = 0;
    RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.2, marker );
    marker.scale.x = 0.03;
    pub_pgraph.publish( marker );

}

void NodeDataManager::publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b )
{
    visualization_msgs::Marker marker ;
    RosMarkerUtils::init_camera_marker( marker, .6 );

    for( int i=0 ; i<w_T_ci.size() ; i++ )
    {
        RosMarkerUtils::setpose_to_marker( w_T_ci[i], marker );
        RosMarkerUtils::setcolor_to_marker( r,g,b, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }
}





void NodeDataManager::publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 )
{
    assert( idx_partition <= w_T_ci.size() );

    visualization_msgs::Marker marker ;
    RosMarkerUtils::init_camera_marker( marker, .6 );

    for( int i=0 ; i<idx_partition ; i++ )
    {
        RosMarkerUtils::setpose_to_marker( w_T_ci[i], marker );
        RosMarkerUtils::setcolor_to_marker( r,g,b, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }

    for( int i=idx_partition ; i<w_T_ci.size() ; i++ )
    {
        RosMarkerUtils::setpose_to_marker( w_T_ci[i], marker );
        RosMarkerUtils::setcolor_to_marker( r1,g1,b1, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }
}

void NodeDataManager::publishPath( vector<Matrix4d> w_T_ci, int start, int end  )
{

    assert( start >=0 && start <= w_T_ci.size() && end >=0 && end <= w_T_ci.size() && start < end  );


    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    for( int i=start ; i<end ; i++ )
    {
        geometry_msgs::PoseStamped pose_stamped;
        // pose_stamped.header = path.header;  //TODO. set correct timestamp at each

        ros::Time stamp;
        bool __status = getNodeTimestamp( i, stamp  );
        assert( __status );
        pose_stamped.header.stamp = stamp;
        pose_stamped.header.frame_id = "world";

        Quaterniond quat( w_T_ci[i].topLeftCorner<3,3>() );


        pose_stamped.pose.orientation.w = quat.w();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();

        pose_stamped.pose.position.x = w_T_ci[i](0,3);
        pose_stamped.pose.position.y = w_T_ci[i](1,3);
        pose_stamped.pose.position.z = w_T_ci[i](2,3);

        path.poses.push_back( pose_stamped );


    }
    pub_path_opt.publish( path );
}

void NodeDataManager::publishPath( nav_msgs::Path path )
{
    pub_path_opt.publish( path );
}


void NodeDataManager::publishLastNEdges( int n )
{
    int start, end;
    edge_mutex.lock();
    int len = loopclosure_edges.size();
    edge_mutex.unlock();

    if( n<= 0 )
        start = 0;
    else
        start = max( 0, len-n );

    end = len;

    for( int i=start; i<end ; i++ )
    {
        edge_mutex.lock();
        int idx_node_prev = loopclosure_edges[i].first ;
        int idx_node_curr = loopclosure_edges[i].second;
        edge_mutex.unlock();

        node_mutex.lock();
        Vector3d w_t_prev = node_pose[ idx_node_prev ].col(3).head(3);
        Vector3d w_t_curr = node_pose[ idx_node_curr ].col(3).head(3);
        node_mutex.unlock();


        // make a line marker
        visualization_msgs::Marker marker;
        RosMarkerUtils::init_line_marker( marker, w_t_prev, w_t_curr );
        marker.id = i;
        marker.ns = "loop_closure_edges";
        RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.2, marker );

        pub_pgraph.publish( marker );
    }
}

/////////////////////// Utility
// Loop over each node and return the index of the node which is clossest to the specified stamp
int NodeDataManager::find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp )
{
  ros::Duration diff;
  for( int i=0 ; i<global_nodes_stamps.size() ; i++ )
  {
    diff = global_nodes_stamps[i] - stamp;

    // cout << i << " "<< diff.sec << " " << diff.nsec << endl;

    if( diff < ros::Duration(0.0001) && diff > ros::Duration(-0.0001) ){
      return i;
    }
  }//TODO: the duration can be a fixed param. Basically it is used to compare node timestamps.
  // ROS_INFO( "Last Diff=%d:%d. Cannot find specified timestamp in nodelist. ", diff.sec,diff.nsec);
  return -1;
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


////////////////////// Save Data to file for analysis //////////////

void NodeDataManager::_print_info_on_npyarray( const cnpy::NpyArray& arr )
{

    // #Dimensions
    cout << "arr.shape.size()" << arr.shape.size() << endl;

    // X.shape
    int raw_data_size = 1;
    for( int i=0 ; i< arr.shape.size() ; i++ ){
            cout << arr.shape[i] << ", ";
            raw_data_size *= arr.shape[i];
    }
    cout << endl;

    // Ordering
    cout << "arr.fortran_order " << arr.fortran_order << endl;

    // Raw data
    double * loaded_data = (double*) arr.data;
    for( int i=0 ; i<raw_data_size ; i++ )
        cout << loaded_data[i] << " ";
}

bool NodeDataManager::loadFromDebug( const string& base_path, const vector<bool>& edge_mask )
{
    cout << "##########################################\n";
    cout << " NodeDataManager::loadFromDebug : "<< base_path << endl;
    cout << "##########################################\n";

    cout << "edge_mask.size()="<< edge_mask.size() << endl;
    for( int i=0 ; i<edge_mask.size() ; i++ )
        cout << edge_mask[i];
    cout << endl;

    node_mutex.lock();
    edge_mutex.lock();
    reset_node_info_data();
    reset_edge_info_data();

    //
    // Load Data on Nodes
    // npz_t := std::map<std::string, NpyArray>;
    cnpy::npz_t my_npz = cnpy::npz_load( base_path+"/graph_data.npz" );
    // cout << "Contains files: \n";
    // for( cnpy::npz_t::iterator i=my_npz.begin() ; i!= my_npz.end() ; i++ )
    // {
    //     cout << i->first << endl;
    // }

    reset_node_info_data();
    for( int i=0 ; ; i++ )
    {
        if( my_npz.count(  "w_T_"+to_string(i) ) == 0  )
            break;

        // w_T_c
        auto arr = my_npz[ "w_T_"+to_string(i) ];
        // _print_info_on_npyarray( arr );
        // Map<Matrix4d> w_T_c( (double*)arr.data );
        Matrix4d w_T_c( (double*) arr.data );

        node_pose.push_back( w_T_c );
    }
    cout << "Loaded " << node_pose.size() << " nodes" << endl;



    //
    // Load Data on Edges
    reset_edge_info_data();



    // a) Edge Idx
    auto arr0 = my_npz[ "loopclosure_edges" ]; // this is of size 2E
    auto arr1 = my_npz[ "loopclosure_edges_goodness" ]; // this is of size 2E
    assert( arr0.shape.size() == 1 && arr1.shape.size() == 1 );
    assert( arr0.shape[0]/2 == arr1.shape[0]);
    int nloops = arr0.shape[0]/2;
    cout << "There are "<< nloops << " loops\n";
    int * loaded_data = (int*) arr0.data;

    assert( edge_mask.size() == 0 || edge_mask.size() == nloops );

    for( int i=0 ; i<nloops; i++ )
    {
        // cout <<i << " " << loaded_data[2*i] << "<-->" << loaded_data[2*i+1] << endl;
        if( edge_mask.size() > 0 && edge_mask[i] == false )
            continue;// dont load the edge if edge_mask has non zero size and mask is true.



        std::pair<int,int> p;
        p.first = loaded_data[2*i];
        p.second = loaded_data[2*i+1];
        this->loopclosure_edges.push_back( p );
    }
    // _print_info_on_npyarray( arr0 );


    // b) Goodness
    double * goodness_loaded_data = (double*) arr1.data;
    for( int i=0 ; i<nloops ; i++ )
    {
        if( edge_mask.size() > 0 && edge_mask[i] == false )
            continue;// dont load the edge if edge_mask has non zero size and mask is true.

        this->loopclosure_edges_goodness.push_back( goodness_loaded_data[i] );
    }


    // c) p_T_c
    for( int i=0 ; ; i++ )
    {
        if( my_npz.count( "p_T_c__"+to_string(i) ) == 0  )
            break;

        if( edge_mask.size() > 0 && edge_mask[i] == false )
            continue;// dont load the edge if edge_mask has non zero size and mask is true.

        auto arr_ed_pose = my_npz[  "p_T_c__"+to_string(i) ];

        Matrix4d p_T_c( (double*)  arr_ed_pose.data );

        this->loopclosure_p_T_c.push_back( p_T_c );
    }
    cout << "Loaded "<< loopclosure_p_T_c.size() << " relative poses for edges (computed)\n";

    // assert( nloops == loopclosure_p_T_c.size() );

    edge_mutex.unlock();
    node_mutex.unlock();


    // Display All
    assert( loopclosure_edges.size() == loopclosure_edges_goodness.size() );
    assert( loopclosure_edges.size() == loopclosure_p_T_c.size() );
    for( int i=0 ; i<loopclosure_edges.size() ; i++ )
    {
        cout << i << " " << loopclosure_edges[i].first << "<-->" << loopclosure_edges[i].second << " ";
        cout << "weight=" << loopclosure_edges_goodness[i] << " ";
        cout << "p_T_c="<< PoseManipUtils::prettyprintMatrix4d( loopclosure_p_T_c[i] );
        cout << endl;
    }
    cout << "Done Loading Pose Graph\n";

}

bool NodeDataManager::saveForDebug( const string& base_path )
{
    cout << "##########################################\n";
    cout << " NodeDataManager::saveForDebug : "<< base_path << endl;
    cout << "##########################################\n";


    // ofstream myfile;
    // myfile.open( base_path+"/test.txt" );
    // myfile << "Test file\n";
    // myfile.close();


    //
    // Write Info on Nodes
    vector<unsigned int> shape;

    /* // examples with cnpy
    // Remember than eigen internally stores data as column major,
    cout << "node_pose_0\n" << node_pose[0] << endl;
    shape = {4,4};
    cnpy::npy_save( base_path+"/graph_data.npy", node_pose[0].data(),  &shape[0], 2, "w" );
    cnpy::npz_save( base_path+"/graph_data.npz", "w_T_0", node_pose[0].data(),  &shape[0], 2, "w" );


    // load the saved data
    cnpy::NpyArray arr = cnpy::npy_load( base_path+"/graph_data.npy" );
    // _print_info_on_npyarray( arr );

    // Matrix4d M;
    Map<Matrix4d> M( (double*)arr.data );
    cout << "M\n" << M << endl;
    */
    int hash_nodes = getNodeLen();
    int hash_edges = getEdgeLen();

    shape = {4,4};
    node_mutex.lock();
    cnpy::npz_save( base_path+"/graph_data.npz", "w_T_0", node_pose[0].data(),  &shape[0], 2, "w" );
    for( int i=1 ; i<hash_nodes ; i++ )
    {
        cnpy::npz_save( base_path+"/graph_data.npz", "w_T_"+to_string(i), node_pose[i].data(),  &shape[0], 2, "a" );
    }
    cout << "written "<< hash_nodes << " pose to .npz\n";

    //TODO Also save node_pose_timestamps;
    shape = {6,6};
    for( int i=0 ; i<hash_nodes ; i++ )
    {
        cnpy::npz_save( base_path+"/graph_data.npz", "cov_w_T_"+to_string(i), node_pose_covariance[i].data(), &shape[0], 2, "a"  );
    }
    node_mutex.unlock();



    //
    // Write Info on Edges
    edge_mutex.lock();
    vector<int> lp_edges;
    shape = {4,4};
    for( int i=0 ; i<loopclosure_edges.size() ; i++ )
    {
        lp_edges.push_back( loopclosure_edges[i].first );
        lp_edges.push_back( loopclosure_edges[i].second );

        cnpy::npz_save( base_path+"/graph_data.npz", "p_T_c__"+to_string(i), loopclosure_p_T_c[i].data(),  &shape[0], 2, "a" );

        cout << i << ":" <<  loopclosure_edges[i].first << "<-->" <<  loopclosure_edges[i].second << "; ";
        cout << "goodness=" << loopclosure_edges_goodness[i] << "; ";
        cout << PoseManipUtils::prettyprintMatrix4d(  loopclosure_p_T_c[i] );
        cout << "\n";
    }

    shape = {lp_edges.size()};
    cnpy::npz_save( base_path+"/graph_data.npz", "loopclosure_edges",  lp_edges.data(), &shape[0], 1, "a" );
    shape = {loopclosure_edges_goodness.size()};
    cnpy::npz_save( base_path+"/graph_data.npz", "loopclosure_edges_goodness",  loopclosure_edges_goodness.data(), &shape[0], 1, "a" );

    edge_mutex.unlock();
    cout << "written "<< hash_edges << " loopedges to .npz\n";


    // ---:files:---
    // w_T_i \forall i=1 ... N
    // p_T_c__i \forall i=1 ... E
    // loopclosure_edges. 2E sized vector
    // loopclosure_edges_goodness. E sized vector


    cout << "Done@\n";
    return true;

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
    cout << "Write : "<< base_path+"/log_posegraph.json"<< endl;
    std::ofstream outf(base_path+"/log_posegraph.json");
    if( !outf.is_open() ) {
        cout << "[ERROR saveAsJSON] Cannot open file\n";
        return false;
    }

    outf << std::setw(4) << all_info << std::endl;
    return true;


}
