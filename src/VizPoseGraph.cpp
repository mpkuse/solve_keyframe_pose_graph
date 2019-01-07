#include "VizPoseGraph.h"


void VizPoseGraph::setVisualizationPublisher( const ros::Publisher& pub )
{
  // usually was "/mish/pose_nodes"
  pub_pgraph = pub;
}

// TODO removal
void VizPoseGraph::setPathPublisher( const ros::Publisher& pub )
{
  // usually was "/mish/pose_nodes"
  pub_path_opt = pub;
}




void VizPoseGraph::publishLastNNodes( int n )
{
    visualization_msgs::Marker marker ;
    RosMarkerUtils::init_camera_marker( marker, .8 );

    int len = manager->getNodeLen();

    int start, end;
    if( n<= 0 )
        start = 0;
    else
        start = max(0,len-n);

    end = len;

    for( int i=start ; i<end ; i++ )
    {
        // node_mutex.lock();
        // Matrix4d w_T_c = node_pose[i];
        const Matrix4d w_T_c = manager->getNodePose(i);
        // node_mutex.unlock();
        // cout << "publish "<< i << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
        RosMarkerUtils::setpose_to_marker( w_T_c, marker );
        RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, marker );
        marker.id = i;
        marker.ns = "vio_kf_pose";

        pub_pgraph.publish( marker );
    }
}



void VizPoseGraph::publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b )
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



void VizPoseGraph::publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1, bool enable_camera_visual  )
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




void VizPoseGraph::publishEdgesAsLineArray( int n )
{
    int start, end;
    int len = manager->getEdgeLen();

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
        auto paur = manager->getEdgeIdxInfo( i );
        int idx_node_prev = paur.first ;
        int idx_node_curr = paur.second;

        Vector3d w_t_prev = manager->getNodePose(idx_node_prev).col(3).head(3);
        Vector3d w_t_curr = manager->getNodePose(idx_node_curr).col(3).head(3);

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


void VizPoseGraph::publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b )
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




void VizPoseGraph::publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 )
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




void VizPoseGraph::publishPath( const vector<Matrix4d>& w_T_ci, int start, int end  )
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
        bool __status = manager->getNodeTimestamp( i, stamp  );
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


void VizPoseGraph::publishPath( const nav_msgs::Path& path )
{
    pub_path_opt.publish( path );
}



void VizPoseGraph::publishLastNEdges( int n )
{
    int start, end;
    int len = manager->getEdgeLen();

    if( n<= 0 )
        start = 0;
    else
        start = max( 0, len-n );

    end = len;

    for( int i=start; i<end ; i++ )
    {
        auto paur = manager->getEdgeIdxInfo(i);
        int idx_node_prev = paur.first;
        int idx_node_curr = paur.second;

        Vector3d w_t_prev = manager->getNodePose(idx_node_prev).col(3).head(3);
        Vector3d w_t_curr = manager->getNodePose(idx_node_curr).col(3).head(3);


        // make a line marker
        visualization_msgs::Marker marker;
        RosMarkerUtils::init_line_marker( marker, w_t_prev, w_t_curr );
        marker.id = i;
        marker.ns = "loop_closure_edges";
        RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.2, marker );

        pub_pgraph.publish( marker );
    }
}
