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
    node_mutex.lock();
    node_timestamps.push_back( msg->header.stamp );
    node_pose.push_back( w_T_cam );
    node_pose_covariance.push_back( Cov );
    node_mutex.unlock();





}

void NodeDataManager::loopclosure_pose_callback( const nap::NapMsg::ConstPtr& msg  )
{
    // ROS_INFO( "NodeDataManager::loopclosure_pose_callback");
    // Add a new edge ( 2 node*)


    ros::Time t_c = msg->c_timestamp;
    ros::Time t_p = msg->prev_timestamp;
    int op_mode = msg->op_mode;
    double goodness = (double)msg->goodness;
    assert( op_mode == 30 );

    // retrive rel-pose  p_T_c
    Matrix4d p_T_c = Matrix4d::Zero();
    Quaterniond quat( msg->p_T_c.orientation.w, msg->p_T_c.orientation.x, msg->p_T_c.orientation.y, msg->p_T_c.orientation.z );
    p_T_c.topLeftCorner<3,3>() = quat.toRotationMatrix();
    p_T_c(0,3) = msg->p_T_c.position.x;
    p_T_c(1,3) = msg->p_T_c.position.y;
    p_T_c(2,3) = msg->p_T_c.position.z;
    p_T_c(3,3) = 1.0;


    // Lock
    node_mutex.lock() ;

    // loop up t_c in node_timestamps[]
    int index_t_c = find_indexof_node(node_timestamps, t_c );

    // loop up t_p in node_timestamps
    int index_t_p = find_indexof_node(node_timestamps, t_p );

    // Unlock
    node_mutex.unlock();

    cout << "[NodeDataManager] Rcvd NapMsg " << index_t_c << "<--->" << index_t_p << endl;
    assert( t_c > t_p );

    std::pair<int,int> closure_edge;
    closure_edge.first = index_t_p;
    closure_edge.second = index_t_c;

    edge_mutex.lock();
    loopclosure_edges.push_back( closure_edge );
    loopclosure_edges_goodness.push_back( goodness );
    loopclosure_p_T_c.push_back( p_T_c );
    edge_mutex.unlock();



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
    init_camera_marker( marker, .8 );

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
        setpose_to_marker( w_T_c, marker );
        setcolor_to_marker( 0.0, 1.0, 0.0, marker );
        marker.id = i;
        marker.ns = "vio_kf_pose";

        pub_pgraph.publish( marker );
    }
}

void NodeDataManager::publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b )
{
    // this is a cost effective way to visualize camera path
    visualization_msgs::Marker marker;
    init_line_marker( marker );

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x *= 0.01;

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
    assert( idx_partition < w_T_ci.size() );


    // this is a cost effective way to visualize camera path
    visualization_msgs::Marker marker;
    init_line_marker( marker );

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
        init_camera_marker( marker2, 10 );
        setpose_to_marker( w_T_ci[last_idx], marker2 );
        setcolor_to_marker( r,g,b, marker2 );
        marker2.scale.x = 0.02;
        marker2.id = 0;
        marker2.ns = ns+string("_cam_visual");
        pub_pgraph.publish( marker2 );
    }

}




void NodeDataManager::publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b )
{
    visualization_msgs::Marker marker ;
    init_camera_marker( marker, .6 );

    for( int i=0 ; i<w_T_ci.size() ; i++ )
    {
        setpose_to_marker( w_T_ci[i], marker );
        setcolor_to_marker( r,g,b, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }
}





void NodeDataManager::publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 )
{
    assert( idx_partition < w_T_ci.size() );

    visualization_msgs::Marker marker ;
    init_camera_marker( marker, .6 );

    for( int i=0 ; i<idx_partition ; i++ )
    {
        setpose_to_marker( w_T_ci[i], marker );
        setcolor_to_marker( r,g,b, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }

    for( int i=idx_partition ; i<w_T_ci.size() ; i++ )
    {
        setpose_to_marker( w_T_ci[i], marker );
        setcolor_to_marker( r1,g1,b1, marker );
        marker.id = i;
        marker.ns = ns; //"opt_kf_pose";

        pub_pgraph.publish( marker );
    }
}

void NodeDataManager::publishPath( vector<Matrix4d> w_T_ci  )
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    for( int i=0 ; i<w_T_ci.size() ; i++ )
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path.header;  //TODO. set correct timestamp at each

        Quaterniond quat( w_T_ci[i].topLeftCorner<3,3>() );


        pose_stamped.pose.orientation.w = quat.w();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();

        pose_stamped.pose.position.x = w_T_ci[i](0,3);
        pose_stamped.pose.position.y = w_T_ci[i](1,3);
        pose_stamped.pose.position.z = w_T_ci[i](2,3);

        path.poses.push_back( pose_stamped );


        pub_path_opt.publish( path );
    }
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
        init_line_marker( marker, w_t_prev, w_t_curr );
        marker.id = i;
        marker.ns = "loop_closure_edges";
        setcolor_to_marker( 0.0, 1.0, 0.2, marker );

        pub_pgraph.publish( marker );
    }
}


// cam_size = 1: means basic size. 1.5 will make it 50% bigger.
void NodeDataManager::init_camera_marker( visualization_msgs::Marker& marker, float cam_size )
{
     marker.header.frame_id = "world";
     marker.header.stamp = ros::Time::now();
     marker.action = visualization_msgs::Marker::ADD;
     marker.color.a = .7; // Don't forget to set the alpha!
     marker.type = visualization_msgs::Marker::LINE_LIST;
    //  marker.id = i;
    //  marker.ns = "camerapose_visual";

     marker.scale.x = 0.003; //width of line-segments
     float __vcam_width = 0.07*cam_size;
     float __vcam_height = 0.04*cam_size;
     float __z = 0.1*cam_size;




     marker.points.clear();
     geometry_msgs::Point pt;
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );

     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );


     // TOSET
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;
}

void NodeDataManager::setpose_to_marker( const Matrix4d& w_T_c, visualization_msgs::Marker& marker )
{
    Quaterniond quat( w_T_c.topLeftCorner<3,3>() );
    marker.pose.position.x = w_T_c(0,3);
    marker.pose.position.y = w_T_c(1,3);
    marker.pose.position.z = w_T_c(2,3);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
}

void NodeDataManager::setcolor_to_marker( float r, float g, float b, visualization_msgs::Marker& marker  )
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
}


void NodeDataManager::init_line_marker( visualization_msgs::Marker &marker )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.005;

    marker.points.clear();

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}

void NodeDataManager::init_line_marker( visualization_msgs::Marker &marker, const Vector3d& p1, const Vector3d& p2 )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.005;

    marker.points.clear();
    geometry_msgs::Point pt;
    pt.x = p1(0);
    pt.y = p1(1);
    pt.z = p1(2);
    marker.points.push_back( pt );
    pt.x = p2(0);
    pt.y = p2(1);
    pt.z = p2(2);
    marker.points.push_back( pt );

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

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
int NodeDataManager::getNodeLen()
{
    node_mutex.lock();
    int n = node_pose.size();
    node_mutex.unlock();
    return n;
}

int NodeDataManager::getEdgeLen()
{
    edge_mutex.lock();
    int n = loopclosure_edges.size();
    edge_mutex.unlock();
    return n;
}

// returns w_T_cam
bool NodeDataManager::getNodePose( int i, Matrix4d& w_T_cam )
{
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

bool NodeDataManager::getNodeCov( int i, Matrix<double,6,6>& cov )
{
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

// return p_T_c
bool NodeDataManager::getEdgePose( int i, Matrix4d& p_T_c )
{
    if( i>=0 && i<loopclosure_edges.size() )
    {
        p_T_c = loopclosure_p_T_c[i];
        return true;
    }
    return false;
}

// edge idx info
bool NodeDataManager::getEdgeIdxInfo( int i, std::pair<int,int>& p )
{
    bool status;
    if( i>=0 && i<loopclosure_edges.size() ) {
        p = loopclosure_edges[i];
        return true;
    }
    return false;
}
