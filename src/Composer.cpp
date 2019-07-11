#include "Composer.h"


// #define __Composer__pose_assember_thread( msg ) msg;
#define __Composer__pose_assember_thread( msg ) ;

// #define __Composer__pose_assember_thread_posedebug( msg ) msg;
#define __Composer__pose_assember_thread_posedebug( msg ) ;

void Composer::pose_assember_thread( int looprate )
{
    cout << TermColor::GREEN() << "Start `pose_assember_thread` @ " << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );

    // After the data is acquired using slam and manager, a deep copy is made into this object's global_jmb and global_lmb
    map<int, vector<Matrix4d> > jmb; // key: world, value: vector of poses
    vector< Matrix4d > lbm_fullpose; // a corrected poses. Same index as the node. These are used for loopedges.
    ElapsedTime elp;

    ros::Rate rate( looprate );
    while( b_pose_assember )
    {

        if( manager->getNodeLen() == 0 )
        {
            rate.sleep();
            continue;
        }

        jmb.clear();
        lbm_fullpose.clear();

        int latest_pose_worldid = -1;
        int ____solvedUntil = slam->solvedUntil(); //note: solvedUntil is the index until which posegraph was solved
        int ____solvedUntil_worldid =  manager->which_world_is_this( manager->getNodeTimestamp(____solvedUntil) );
        bool ____solvedUntil_worldid_is_neg = false;
        if( ____solvedUntil_worldid < 0 ) { /*____solvedUntil_worldid = -____solvedUntil_worldid - 1;*/ ____solvedUntil_worldid_is_neg=true; }

        __Composer__pose_assember_thread(
        cout << "____solvedUntil=" << ____solvedUntil << "\t";
        cout << "____solvedUntil_worldid=" << ____solvedUntil_worldid << "\t";
        cout << "__slam->get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status=" << slam->get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status() << "\t";
        cout << endl;
        )



        //---------- for i = 0:manager->getNodeLen() -----//
        //                  if( i>=0  i<= ____solvedUntil )
        //                              ""--do--""
        //                  if( i>(____solvedUntil)  )
        //                              ""--do--""
        elp.tic();
        for( int i=0 ; i<manager->getNodeLen() ; i++ )
        {
            int world_id = manager->which_world_is_this( manager->getNodeTimestamp(i) );
            int setID_of_worldID = manager->getWorldsConstPtr()->find_setID_of_world_i( world_id  );
            __Composer__pose_assember_thread_posedebug(
            cout << "\t%% i=" << i << " world#" << world_id << " setID_of_worldID=" << setID_of_worldID << endl;
            )

            if( i>=0 && i<= ____solvedUntil )
            {
                __Composer__pose_assember_thread_posedebug( cout << "in i>=0 && i<= ____solvedUntil\n" );
                Matrix4d w_T_c;
                // If the optimized pose exists use that else use the odometry pose
                int from_slam_or_from_odom = -1; // 1: from slam ; 2: from odom ; 3: kidnapped node
                if( world_id >= 0 ) {
                    if( slam->nodePoseExists(i) /*___slam_nodePoseExists_i*/ ) {
                        w_T_c = slam->getNodePose( i );
                        /*w_T_c = ___slam_getNodePose_i;*/
                        // cerr << "w_T_c_optimized=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
                        from_slam_or_from_odom = 1;
                    } else {
                        if( manager->nodePoseExists(i )  ) {
                            w_T_c = manager->getNodePose( i );
                            // cerr << "w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
                            from_slam_or_from_odom = 2;



                        }
                    }
                } else {
                    // kidnapped worlds viz, -1, -2 etc.
                    // use the last pose and add the odometry to it
                    // TODO
                    from_slam_or_from_odom = 3;

                    int last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 );
                    Matrix4d w_T_last;
                    w_T_last = *(jmb.at( -world_id - 1  ).rbegin());
                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_T_c = w_T_last * last_M_i;
                }

                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();

                jmb[ world_id ].push_back( w_T_c );
                // lbm.push_back( w_T_c.col(3).topRows(3) );
                lbm_fullpose.push_back( w_T_c );
                latest_pose_worldid = world_id;

                __Composer__pose_assember_thread_posedebug(
                cout << TermColor::RED() << i << ":" <<  world_id << "  (from_slam_or_from_odom=" << from_slam_or_from_odom << " w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl << TermColor::RESET();
                )


            }


            if( i>(____solvedUntil)  )
            {
                __Composer__pose_assember_thread_posedebug( cout << "in i>(____solvedUntil)\n" );

                int last_idx=-1;
                Matrix4d w_TM_i;
                #if 1 // added during loadStateFromDisk implementation
                int w_TM_i_from_mgr_or_from_slam = -1; // 0: mgr, 1: slam
                #endif

                if( ____solvedUntil == 0 ) {
                    w_TM_i = manager->getNodePose( i );
                    #if 1 // added during loadStateFromDisk implementation
                    w_TM_i_from_mgr_or_from_slam = 0;
                    #endif
                } else {
                    if( world_id >= 0 && ____solvedUntil_worldid == world_id ) {
                        last_idx = ____solvedUntil;}
                    else if( world_id >=0 && ____solvedUntil_worldid != world_id  ) {
                        w_TM_i = manager->getNodePose( i );
                    }
                    else if( world_id < 0 ) {
                        // this is the kidnaped node
                        last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 ); // only this in working code

                        if( !( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen()) ) {
                            cout << "[Composer::pose_assember_thread]ERROR. last_idx=" << last_idx << endl;
                            manager->getWorldsConstPtr()->print_summary( 2);
                            manager->print_worlds_info(2);
                            assert( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen() );
                        }

                        w_TM_i = *(jmb[ -world_id-1 ].rbegin()) * ( manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i ) ) ;
                        last_idx = -1;

                    } else {
                        cout << "\n[Composer::pose_assember_thread] impossivle\n";
                        exit(2);
                    }

                }


                if( last_idx >= 0 ) {
                    Matrix4d w_T_last;
                    if( slam->nodePoseExists(last_idx) )
                        w_T_last = slam->getNodePose(last_idx );
                    else
                        w_T_last = manager->getNodePose(last_idx );

                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_TM_i = w_T_last * last_M_i;
                }


                #if 1
                // added during loadStateFromDisk implementation
                // idea is always set ws_T_c and not the usual w_T_c
                //  ws_T_c := ws_T_w * w_T_c
                if( world_id != setID_of_worldID && w_TM_i_from_mgr_or_from_slam==0 ) {
                    __Composer__pose_assember_thread_posedebug(
                    cout << "[world_id != setID_of_worldID]i=" << i << " world#" << world_id << " setID_of_worldID=" << setID_of_worldID << endl;
                    );
                    if( manager->getWorldsConstPtr()->is_exist(setID_of_worldID,world_id) ) {
                        __Composer__pose_assember_thread_posedebug(
                        cout << "[world_id != setID_of_worldID]relative pose exist beween these worlds.\n";
                        );
                        Matrix4d ws_T_w = manager->getWorldsConstPtr()->getPoseBetweenWorlds(setID_of_worldID,world_id);
                        Matrix4d ws_T_c = ws_T_w * w_TM_i;
                        w_TM_i = ws_T_c;
                    } else {
                        __Composer__pose_assember_thread_posedebug( cout << "[world_id != setID_of_worldID]relative pose doesnt exist beween these worlds.\n"; );
                    }
                }

                #endif // end of addition during oadStateFromDisk implementation


                if( jmb.count( world_id ) ==  0 ) {
                    __Composer__pose_assember_thread_posedebug(
                    cout << "jmb[" << world_id << "] = vector<Matrix4d>() \n";
                    )
                    jmb[ world_id ] = vector<Matrix4d>();
                }

                jmb[ world_id ].push_back( w_TM_i );
                // lbm.push_back( w_TM_i.col(3).topRows(3) );
                lbm_fullpose.push_back( w_TM_i );
                latest_pose_worldid = world_id;

                __Composer__pose_assember_thread_posedebug(
                cout << i << ":" << world_id << "  w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << "  last_idx="<<  last_idx << endl;
                )

            }

        }

        __Composer__pose_assember_thread(
        cout << TermColor::BLUE() << "[Composer::pose_assember_thread]main for loop took : " << elp.toc_milli() << " ms" << TermColor::RESET() << endl;
        )


        elp.tic();
        {
            // jmb ===>(data copy) global_jmb. Deep copy needed
            __Composer__pose_assember_thread(
            cout <<"[Composer::pose_assember_thread]// jmb ===>(data copy) global_jmb. Deep copy needed\n";
            )
            std::lock_guard<std::mutex> lk(mx);

            // Deep copy std::map
            this->global_jmb.clear();
            for( auto it=jmb.begin() ; it!= jmb.end() ; it++ ) // loop over worlds
            {
                __Composer__pose_assember_thread(
                cout << "\t[Composer::pose_assember_thread]process world#" << it->first << " it has " << it->second.size() << " poses" << endl;
                )
                // loop over poses in this world
                for( auto vec_it = it->second.begin() ; vec_it != it->second.end() ; vec_it++ )
                {
                    this->global_jmb[ it->first ].push_back( *vec_it );
                }
            }

            // Deep copy std::vector
            this->global_lmb.clear();
            __Composer__pose_assember_thread(
            cout << "[Composer::pose_assember_thread]lbm_fullpose.size() = " << lbm_fullpose.size() << endl;
            )
            for( auto it=lbm_fullpose.begin() ; it!=lbm_fullpose.end() ; it++ )
            {
                this->global_lmb.push_back( *it );
            }

            // latest
            __Composer__pose_assember_thread(
            cout << "[Composer::pose_assember_thread]setting this->global_latest_pose_worldid := " << latest_pose_worldid << endl;
            )
            this->global_latest_pose_worldid = latest_pose_worldid;

        }
        __Composer__pose_assember_thread(
        cout << TermColor::BLUE() << "[Composer::pose_assember_thread]deepcopy took : " << elp.toc_milli() << " ms" << TermColor::RESET() << endl;
        )

        rate.sleep();
    }

    cout << TermColor::RED() << "Finished `pose_assember_thread`\n" << TermColor::RESET();
}


int Composer::get_last_known_camerapose( Matrix4d& w_T_lastcam, ros::Time& stamp_of_it )
{
    std::lock_guard<std::mutex> lk(mx);
    const int sz = global_lmb.size();
    if( sz == 0 )
        return -1;

    auto it = global_lmb.rbegin();
    w_T_lastcam = *(it);

    stamp_of_it = manager->getNodeTimestamp( sz-1 );
    return sz-1;
}

// #define __Composer_bf_traj_publish_thread( msg ) msg;
#define __Composer_bf_traj_publish_thread( msg ) ;
void Composer::bf_traj_publish_thread( int looprate ) const
{
    //--- Options
    const int options_line_color_style = 10; // should be either 10 or 12. 10:color by WorldID; 12:  //color by world setID
    const float options_linewidth_multiplier  = 3; // thickness of the line

    //---

    //          Note: global_jmb's keys are `worldIDs` and global_lmb's values are `vector<Matrix4d>& w_T_ci`
    cout << TermColor::GREEN() << "start `Composer::bf_traj_publish_thread` @ " << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );
    ros::Rate rate(looprate);
    bool published_axis = false;

    static thread_local std::mt19937 generator;
    std::uniform_int_distribution<int> distribution(0,100);

    int x_loop_count = 0;
    while( b_bf_traj_publish )
    {
        rate.sleep();
        x_loop_count++;
        __Composer_bf_traj_publish_thread(
        cout <<  "[Composer::bf_traj_publish_thread] publish : global_lmb.size()=" << global_lmb.size() << "\t n_worlds aka. global_jmb.size()=" <<  global_jmb.size() << endl;
        )

        { // this braces for mutex
            // acquire lock and publish using global_lmb and global_jmb
            std::lock_guard<std::mutex> lk(mx);

            if( global_jmb.size() == 0 )
            {
                // cout << "[Composer::bf_traj_publish_thread] not publishing because jmb.size is zero\n";
                continue;
            }

            bool publish_all = (distribution(generator) < 5 || x_loop_count < 10 )?true:false;


            for( auto it=global_jmb.begin() ; it!=global_jmb.end() ; it++ )
            {
                // 10% of the times publish all worlds, 90% of the time publish only the newest
                if( publish_all == false )
                {
                    if( it->first == this->global_latest_pose_worldid )
                    {
                        //OK
                    }else {
                        continue;
                    }
                }

                string ns = "world#"+to_string( it->first );
                __Composer_bf_traj_publish_thread(
                cout << "[Composer::bf_traj_publish_thread] publish for ns=" << ns << endl;
                )

                float c_r=0., c_g=0., c_b=0.;
                int rng=-1;

                if( options_line_color_style == 10 )
                    rng = it->first; //color by world id WorldID
                else if( options_line_color_style == 12 )
                    rng = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first ); //color by setID
                else {
                    cout << TermColor::RED() << "[Composer::bf_traj_publish_thread] ERROR. invalid option `opt_traj_publisher_options`. expected either 10 or 12.\n";
                    exit( 1 );
                }


                if( rng >= 0 ) {
                    cv::Scalar color = FalseColors::randomColor( rng );
                    c_r = color[2]/255.;
                    c_g = color[1]/255.;
                    c_b = color[0]/255.;
                }


                float offset_x=0., offset_y=0., offset_z=0.;
                #if 0
                int curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first );
                if( curr_set_id >= 0 ) {
                    if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                        // cout << "rng=" << rng << "setids_to_udumbes"<<  setids_to_udumbes.at( rng ) << endl;
                        offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                    }
                } else {
                    curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( -it->first-1);
                    if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                        offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                    }
                }
                #endif

                viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b, options_linewidth_multiplier, offset_x, offset_y, offset_z );

            }


        }

        if( published_axis==false || rand() % 100 == 0 ) {
        // Publish Axis - publish infrequently
        Matrix4d _axis_pose = Matrix4d::Identity();
        viz->publishXYZAxis( _axis_pose, "opt_traj_axis", 0  );
        published_axis = true;
        }
    }

    cout << TermColor::RED() << "Finished `Composer::bf_traj_publish_thread`\n" << TermColor::RESET() << endl;
}



void Composer::cam_visual_publish_thread( int looprate ) const
{
    //--- Options
    const int options_linewidth_multiplier = 3; //default 3
    //---


    cout << TermColor::GREEN() << "start `Composer::cam_visual_publish_thread` @" << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );

    ros::Rate rate(looprate);
    while( b_cam_visual_publish )
    {
        rate.sleep();

        {
            std::lock_guard<std::mutex> lk(mx);
            // cout << "[Composer::cam_visual_publish_thread] global_latest_pose_worldid" << global_latest_pose_worldid << endl;
            if(  global_latest_pose_worldid < 0 )
                continue;
            Matrix4d wi_T_latest = *( global_jmb.at( global_latest_pose_worldid ).rbegin() );
            // cout << "\twi_T_latest = " << PoseManipUtils::prettyprintMatrix4d( wi_T_latest ) << endl;
            float offset_x=0., offset_y=0., offset_z=0.;
            float c_r=0., c_g=0., c_b=0.;
            int rng=-1;
            rng = global_latest_pose_worldid;

            if( rng >= 0 ) {
                cv::Scalar color = FalseColors::randomColor( rng );
                c_r = color[2]/255.;
                c_g = color[1]/255.;
                c_b = color[0]/255.;
            }

            viz->publishCameraVisualMarker( wi_T_latest, "world##", c_r, c_g, c_b,
                    options_linewidth_multiplier , 20,
                    offset_x, offset_y, offset_z );

        }
    }
    cout << TermColor::RED() << "Finished `Composer::cam_visual_publish_thread`\n" << TermColor::RESET() << endl;
}

// #define __Composer__path_publish_thread__(msg) msg;
#define __Composer__path_publish_thread__(msg) ;
void Composer::path_publish_thread( int looprate )
{
    cout << TermColor::GREEN() << "start `Composer::path_publish_thread` @" << looprate << " hz" << TermColor::RESET() << endl;
    cout << "[Composer::path_publish_thread]will use `b_cam_visual_publish` for terminating this thread\n";
    assert( looprate > 0 && looprate < 50 );

    int n, prev_n = 0;
    nav_msgs::Path path_msg;

    string path_topic = string( "adhoc/xpath" );
    ROS_INFO( "[Composer::setup_200hz_publishers] Publish to %s", path_topic.c_str() );
    pub_hz200_marker = nh.advertise<nav_msgs::Path>( path_topic , 1000 );

    ros::Publisher pub___path = nh.advertise<nav_msgs::Path>( path_topic , 1000 );

    ros::Rate rate(looprate);
    while( b_cam_visual_publish )
    {
        rate.sleep();

        {
            __Composer__path_publish_thread__( "---\n");
            std::lock_guard<std::mutex> lk(mx);
            if(  global_latest_pose_worldid < 0 ) {
                __Composer__path_publish_thread__( cout << "global_latest_pose_worldid is negative, so, continue...\n"; )
                continue;
            }

            n = global_lmb.size();
            if( prev_n == n || n == 0 ) {
                __Composer__path_publish_thread__(
                cout << "[Composer::path_publish_thread] nothing new, n same as prev_n=" << n << "\n";
                )
                continue;
            }

            Matrix4d imu_T_cam = manager->get_imu_T_cam();
            Matrix4d wi_T_latest =  global_lmb.at( n-1 ) * imu_T_cam.inverse();

            if( rand()%100 > 2 ) {
                //add to msg
                ros::Time stamp_of_it = manager->getNodeTimestamp( n-1 );
                path_msg.header.stamp = stamp_of_it;
                path_msg.header.frame_id = "world";
                geometry_msgs::PoseStamped pxl;
                pxl.header.stamp = stamp_of_it;
                PoseManipUtils::eigenmat_to_geometry_msgs_Pose( wi_T_latest, pxl.pose );
                path_msg.poses.push_back( pxl );
                __Composer__path_publish_thread__(
                cout << "[Composer::path_publish_thread]push_back " << stamp_of_it<< endl;
                )
            }
            else {
                // reset and set all again
                __Composer__path_publish_thread__(
                    cout << TermColor::iRED() << "[Composer::path_publish_thread]reset and then for h=[0:" << n << ")" <<  TermColor::RESET() << endl;
                )
                path_msg.poses.clear();
                for( int h=0 ; h<n ; h++ )
                {
                    ros::Time stamp_of_it = manager->getNodeTimestamp( h );
                    path_msg.header.stamp = stamp_of_it;
                    path_msg.header.frame_id = "world";
                    geometry_msgs::PoseStamped pxl;
                    pxl.header.stamp = stamp_of_it;
                    auto ___wi_T_c = global_lmb.at( h ) * imu_T_cam.inverse();
                    PoseManipUtils::eigenmat_to_geometry_msgs_Pose( ___wi_T_c, pxl.pose );
                    path_msg.poses.push_back( pxl );
                }
            }
        }

        prev_n = n;
        __Composer__path_publish_thread__( cout << "[Composer::path_publish_thread]publish path, contains: " <<  path_msg.poses.size() << " poses\n"; )
        pub___path.publish( path_msg );


    }
    cout << TermColor::RED() << "Finished `Composer::path_publish_thread`\n" << TermColor::RESET() << endl;
}

#define  __Composer__w0_T_w1_publish_thread( msg ) msg;
// #define  __Composer__w0_T_w1_publish_thread( msg ) ;
void Composer::w0_T_w1_publish_thread( int looprate )
{
    cout << TermColor::GREEN() << "start `Composer::w0_T_w1_publish_thread` @" << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 5 );

    string w0_T_w1_topic = string( "adhoc/w0_T_w1");
    ROS_INFO( "[Composer::setup_200hz_publishers] Publish to %s", w0_T_w1_topic.c_str() );
    ros::Publisher pub_w0_T_w1 = nh.advertise<geometry_msgs::PoseStamped>( w0_T_w1_topic , 1000 );


    ros::Rate rate(looprate);
    while( b_cam_visual_publish )
    {
        rate.sleep();

        __Composer__w0_T_w1_publish_thread( cout << "----w0_T_w1_publish_thread---\n"; )


        // w0_T_w1 if available
        if( manager->getWorldsConstPtr()->is_exist( 0, 1 ) )
        {
            // publish.
            __Composer__w0_T_w1_publish_thread( cout << "[Composer::w0_T_w1_publish_thread]w0_T_w1 exist...pubnlish/....\n"; )
            Matrix4d w0_T_w1 = manager->getWorldsConstPtr()->getPoseBetweenWorlds( 0, 1);
            __Composer__w0_T_w1_publish_thread( cout << "w0_T_w1=" << PoseManipUtils::prettyprintMatrix4d( w0_T_w1 ) << endl; )


            geometry_msgs::PoseStamped msg;
            PoseManipUtils::eigenmat_to_geometry_msgs_Pose( w0_T_w1, msg.pose );
            msg.header.stamp = ros::Time();
            msg.header.frame_id = "w0_T_w1";

            pub_w0_T_w1.publish( msg );
        }
        else {
            __Composer__w0_T_w1_publish_thread( cout << "[Composer::w0_T_w1_publish_thread]w0_T_w1 doesnt exist, so dont publish anything\n"; )
        }


    }

    cout << TermColor::RED() << "Finished `Composer::w0_T_w1_publish_thread`\n" << TermColor::RESET() << endl;

}


// #define __Composer__loopedge_publish_thread(msg) msg;
#define __Composer__loopedge_publish_thread(msg) ;
void Composer::loopedge_publish_thread( int looprate ) const
{
    cout << TermColor::GREEN() << "start `Composer::loopedge_publish_thread` @" << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );

    //--- Options
    const int options_linewidth_multiplier = 3; //default 3
    //---


    ros::Rate rate(looprate);
    int prev_loopedge_len = 0;
    while( b_loopedge_publish )
    {
        rate.sleep();
        int nloopedgfes = manager->getEdgeLen();
        __Composer__loopedge_publish_thread(
        cout << "[Composer::loopedge_publish_thread] nloopedgfes=" << nloopedgfes << endl;
        )



        if( nloopedgfes == prev_loopedge_len ) {
            __Composer__loopedge_publish_thread(
            cout << "no new loopedges\n";
            )
        }
        else {
            __Composer__loopedge_publish_thread(
            cout << "new from idx=[" << prev_loopedge_len << ", " << nloopedgfes <<  ")" << endl;
            )

            {
                std::lock_guard<std::mutex> lk(mx);
                // TODO: in the future publish intra-world loopedges in different color and interworld as different color
                visualization_msgs::Marker linelist_marker;
                RosMarkerUtils::init_line_marker( linelist_marker );
                linelist_marker.ns = "loopedges_on_opt_traj"; linelist_marker.id = 0;
                linelist_marker.color.r = 0.42;linelist_marker.color.g = 0.55;linelist_marker.color.b = 0.14;linelist_marker.color.a = 1.;
                linelist_marker.scale.x = 0.1*options_linewidth_multiplier;


                // 5% of the times publish all
                // 95% of time publish the newest
                // ? TODO

                for( int it=0 ; it<nloopedgfes; it++ ) {
                    auto pair = manager->getEdgeIdxInfo( it );
                    int __a = pair.first;
                    int __b = pair.second;

                    int __a_worldid = manager->which_world_is_this( manager->getNodeTimestamp(__a) );
                    int __a_setid = manager->getWorldsConstPtr()->find_setID_of_world_i( __a_worldid );
                    int __b_worldid = manager->which_world_is_this( manager->getNodeTimestamp(__b) );
                    int __b_setid = manager->getWorldsConstPtr()->find_setID_of_world_i( __b_worldid );
                    #if 0
                    cout << TermColor::CYAN() ;
                    cout << "it=" << it;
                    cout << "__a=" << __a << " __a_worldid=" << __a_worldid << " __a_setid=" << __a_setid << "\t|";
                    cout << "__b=" << __b << " __b_worldid=" << __b_worldid << " __b_setid=" << __b_setid << "\n";
                    cout << TermColor::RESET();
                    #endif


                    Vector3d ____apose = global_lmb[__a].col(3).topRows(3) ;
                    Vector3d ____bpose = global_lmb[__b].col(3).topRows(3) ;

                    RosMarkerUtils::add_point_to_marker(  ____apose, linelist_marker, false );
                    RosMarkerUtils::add_point_to_marker(  ____bpose, linelist_marker, false );
                }
                viz->publishThisVisualMarker( linelist_marker );

            }
        } // else of if( nloopedgfes == prev_loopedge_len )




        prev_loopedge_len = nloopedgfes;
    }
    cout << TermColor::RED() << "Finished `Composer::loopedge_publish_thread`\n" << TermColor::RESET() << endl;
}



// #define __Composer__disjointset_statusimage_publish_thread(msg) msg;
#define __Composer__disjointset_statusimage_publish_thread(msg) ;
void Composer::disjointset_statusimage_publish_thread( int looprate ) const
{
    cout << TermColor::GREEN() << "start `Composer::disjointset_statusimage_publish_thread` @" << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );

    ros::Rate rate(looprate);
    string prev_status_str = "";
    while( b_disjointset_statusimage_publish )
    {
        rate.sleep();

        string curr_status_str = manager->getWorldsConstPtr()->disjoint_set_status();
        if( prev_status_str.compare(curr_status_str) == 0 ) {
            // same as old, no point publishing
            __Composer__disjointset_statusimage_publish_thread(
            cout << "[Composer::disjointset_statusimage_publish_thread] No change in status, so dont publish image." << endl;
            )
            continue;
        }
        prev_status_str = curr_status_str;
        __Composer__disjointset_statusimage_publish_thread(
        cout << "[Composer::disjointset_statusimage_publish_thread]" << curr_status_str << endl;
        )

        cv::Mat im_disp;
        // manager->getWorldsConstPtr()->disjoint_set_status_image(im_disp); // will get bubles as well as the text
        manager->getWorldsConstPtr()->disjoint_set_status_image(im_disp, true, false); // only bubles

        #if 1 // set this to zero to imshow the image. helpful for debugging.
        __Composer__disjointset_statusimage_publish_thread(
        cout << "[Composer::disjointset_statusimage_publish_thread] publishImage\n";
        )
        viz->publishImage( im_disp );
        #else
        cv::imshow( "disjoint_set_status_image" , im_disp );
        cv::waitKey(30);
        #endif
    }

    cout << TermColor::RED() << "Finished `Composer::disjointset_statusimage_publish_thread`\n" << TermColor::RESET() << endl;
}




//------ Publish body pose @200Hz ------//

void Composer::setup_200hz_publishers()
{
    // pubx =
    // cmpr->set_200hz_marker_pub( pubx )
    string marker_topic = string( "hz200/visualization_marker");
    ROS_INFO( "[Composer::setup_200hz_publishers] Publish to %s", marker_topic.c_str() );
    pub_hz200_marker = nh.advertise<visualization_msgs::Marker>( marker_topic , 1000 );


    // geometry_msgs::Pose
    string pose_topic = string( "hz200/pose");
    ROS_INFO( "[Composer::setup_200hz_publishers] Publish to %s", pose_topic.c_str() );
    pub_hz200_pose = nh.advertise<geometry_msgs::Pose>( pose_topic , 1000 );

    // geometry_msgs::PoseStamped
    string posestamped_topic = string( "hz200/posestamped");
    ROS_INFO( "[Composer::setup_200hz_publishers] Publish to %s", posestamped_topic.c_str() );
    pub_hz200_posestamped = nh.advertise<geometry_msgs::PoseStamped>( posestamped_topic , 1000 );


    // path


}


// #define _Composer__imu_propagate_callback_( msg ) msg;
#define _Composer__imu_propagate_callback_( msg ) ;
void Composer::imu_propagate_callback( const nav_msgs::Odometry::ConstPtr& msg )
{
    _Composer__imu_propagate_callback_(
    cout << TermColor::iBLUE() << "[Composer::imu_propagate_callback] t=" << msg->header.stamp << TermColor::RESET() << endl;
    )

    Matrix4d w_T_imucurr;  // = pose from msg
    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, w_T_imucurr );




    //
    // Last Known camera-pose? - after pose graph solver
    Matrix4d wf_T_camlast;
    ros::Time cam_t;
    int posegraph_nodeidx = get_last_known_camerapose( wf_T_camlast, cam_t );
    if( posegraph_nodeidx < 0  ) {
        _Composer__imu_propagate_callback_(
        cout << TermColor::iYELLOW() << "posegraph_nodeidx was neg, meaning non existant last known cam pose\n" << TermColor::RESET();
        )
        return ;
    }

    _Composer__imu_propagate_callback_(
    cout << "\tLast known camera-pose is at posegraphnode postion=" << posegraph_nodeidx << " is at t=" << cam_t << " which is "<< msg->header.stamp - cam_t << " behind current imu_prop msg\n";
    )


    //
    // extrinsic-calibration value:  imu_T_cam
    if( manager->is_imu_T_cam_available() == false ) {
        _Composer__imu_propagate_callback_(
        cout << TermColor::iYELLOW() << "imu-cam extrinsic is not yet available\n" << TermColor::RESET() << endl;
        )
        return;
    }
    Matrix4d imu_T_cam = manager->get_imu_T_cam();

    //
    // relative pose between when imu at t=now_t and when imu was at t=last_known_t
    Matrix4d w_T_imulast = manager->getNodePose(posegraph_nodeidx) * imu_T_cam.inverse();
    Matrix4d imulast_T_imucurr = w_T_imulast.inverse() * w_T_imucurr;


    //
    // Add this relative pose to `wf_T_camlast`
    Matrix4d wf_T_imucurr = ( wf_T_camlast * imu_T_cam.inverse() ) * imulast_T_imucurr;


    //==============================================================//
    //--------------------done computation....now publixh-----------//
    //==============================================================//
    bool publish_marker = true;
    bool publish_txt_marker = false;


    //
    // Make viz marker and publish
    if( publish_marker ) {
        visualization_msgs::Marker imu_m;
        RosMarkerUtils::init_XYZ_axis_marker( imu_m, 1.0 );
        imu_m.ns = "hz100_imu";
        imu_m.id = 0;
        RosMarkerUtils::setpose_to_marker( wf_T_imucurr, imu_m  );
        pub_hz200_marker.publish( imu_m );
    }

    if( publish_txt_marker ) {
        visualization_msgs::Marker imu_txt;
        RosMarkerUtils::init_text_marker( imu_txt );
        imu_txt.ns = "hz100_imu_txt";
        imu_txt.text = "IMU@200hz";
        imu_txt.scale.z = 0.5;
        RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0, imu_txt );
        RosMarkerUtils::setpose_to_marker( wf_T_imucurr, imu_txt  );
        pub_hz200_marker.publish( imu_txt );
    }


    geometry_msgs::Pose posex;
    PoseManipUtils::eigenmat_to_geometry_msgs_Pose( wf_T_imucurr, posex );

    geometry_msgs::PoseStamped posexstamped;
    posexstamped.header.stamp = msg->header.stamp;
    int _worldID = manager->which_world_is_this( cam_t );
    int _setID_of_worldID = manager->getWorldsConstPtr()->find_setID_of_world_i( _worldID );
    posexstamped.header.frame_id = "pose_in_world#" + std::to_string( _setID_of_worldID );
    posexstamped.pose = posex;

    pub_hz200_pose.publish( posex );
    pub_hz200_posestamped.publish( posexstamped );



}


//------ END Publish body pose @200Hz ------//





bool Composer::saveStateToDisk( string save_dir_path )
{


    //---
    // rm -rf && mkdir
    cout << TermColor::GREEN();
    cout << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    cout << "^^^^^^^^^^    Composer::saveStateToDisk  ^^^^^^^^^^\n";
    cout << "^^^^^^^^^^    DIR=" << save_dir_path ;
    cout << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    cout << TermColor::RESET();


    //---
    // mark as kidnapped and signal end-of-world
    cout << TermColor::iYELLOW() << "manager->curr_kidnap_status="<< manager->curr_kidnap_status() << TermColor::RESET() << ". 0:=unkidnapped  1:=kidnapped" << endl;
    if( manager->curr_kidnap_status() == false ) { //mark as kidnapped when I am not kidnapped
        cout << TermColor::iYELLOW() << "manager->mark_as_kidnapped_and_signal_end_of_world()\n" << TermColor::RESET() ;
        manager->mark_as_kidnapped_and_signal_end_of_world();
    } else {
        cout << "I was already kidnapped when you pressed CTRL+C, so no further action taken\n";
    }
    cout << TermColor::iYELLOW() << "(after marking kidnapped) manager->curr_kidnap_status="<< manager->curr_kidnap_status() << TermColor::RESET() << ". 0:=unkidnapped  1:=kidnapped" << endl;



    //---
    // rm -rf save_folder_name ; mkdir save_folder_name
    string system_cmd0 = string( "rm -rf ") + save_dir_path + " && mkdir "+ save_dir_path;
    const int rm_dir_err0 = RawFileIO::exec_cmd( system_cmd0 );
    if ( rm_dir_err0 == -1 )
    {
        cout << TermColor::RED() << "[Composer::saveStateToDiskr] Cannot mkdir folder: " << save_dir_path << "!\n" << TermColor::RESET() << endl;
        cout << "So not saveing state to disk...return false\n";
        return false;
    }


    //---
    // save pose graph (corrected poses)
    IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    json obj;

    json pose_graph;
    for( int i=0 ; i<global_lmb.size() ; i++ )
    {
        json this_node;
        Matrix4d wTc = global_lmb[i];

        // wj_T_c : Corrected poses
        this_node["w_T_c"]["rows"] = wTc.rows();
        this_node["w_T_c"]["cols"] = wTc.rows();
        std::stringstream ss;
        ss <<  wTc.format(CSVFormat);
        this_node["w_T_c"]["data"] = ss.str();
        this_node["w_T_c"]["data_pretty"] = PoseManipUtils::prettyprintMatrix4d(wTc);


        // j : setID of the world
        // k : worldID of this pose
        int worldid = manager->which_world_is_this( manager->getNodeTimestamp(i) );
        this_node["worldID"] = worldid;
        this_node["setID_of_worldID"] = manager->getWorldsConstPtr()->find_setID_of_world_i( worldid );
        this_node["stampNSec"] = manager->getNodeTimestamp(i).toNSec();
        this_node["seq"] = i;

        pose_graph.push_back( this_node );
    }
    obj["SolvedPoseGraph"] = pose_graph;

    //---
    // Kidnap Timestamps
    obj["KidnapTimestamps"] = manager->kidnap_data_to_json();
    cout << "In len( obj[\"KidnapTimestamps\"][\"kidnap_starts\"] ) = " << obj["KidnapTimestamps"]["kidnap_starts"].size() << endl;
    cout << "In len( obj[\"KidnapTimestamps\"][\"kidnap_ends\"] ) = " << obj["KidnapTimestamps"]["kidnap_ends"].size() << endl;

    //---
    // World poses data and disjoint_set.
    obj["WorldsData"] = manager->getWorldsConstPtr()->saveStateToDisk();


    //---
    // Don't keep worlds unended. look at
    //  a) WorldsData.vec_world_starts.size
    //  b) WorldsData.vec_world_ends.size
    //  c) KidnapTimestamps.kidnap_starts.size
    //  d) KidnapTimestamps.kidnap_ends.size
    //      Every world that has started need to end, every kidnap that has started has to end.
    //      This also implies that `WorldsData.vec_world_starts.size` need to be equal to `WorldsData.vec_world_ends.size`
    //      also `KidnapTimestamps.kidnap_ends.size` need to be equal to `KidnapTimestamps.kidnap_starts.size`
    //           If they are not equal make them equal. by ending the world with last known timestamp of node or in case of kidnap as well.
    // if( obj["KidnapTimestamps"]["kidnap_ends"] )
    int __a__ = obj.at("WorldsData").at("vec_world_starts").size();
    int __b__ = obj.at("WorldsData").at("vec_world_ends").size();
    int __c__ = obj.at("KidnapTimestamps").at("kidnap_starts").size() ;
    int __d__ = obj.at("KidnapTimestamps").at("kidnap_ends").size() ;
    cout << TermColor::BLUE() << "a) WorldsData.vec_world_starts.size" << __a__ << TermColor::RESET() << endl;
    cout << TermColor::BLUE() << "b) WorldsData.vec_world_ends.size" << __b__ << TermColor::RESET() << endl;
    cout << TermColor::BLUE() << "c) KidnapTimestamps.kidnap_starts.size" << __c__ << TermColor::RESET() << endl;
    cout << TermColor::BLUE() << "d) KidnapTimestamps.kidnap_ends.size" << __d__ << TermColor::RESET() << endl;
    auto last_node_timestamp = obj.at("SolvedPoseGraph").rbegin()->at( "stampNSec"); //long int


    //assuming CTRL+C was pressed when status was not KIDNAPED
    // do-1: end the world
    // do-2: start kidnap
    // also do when you begin new bag: end kidnap and start new world from 1st item of bag


    // assuming CTRL+C was pressed when status was KIDNAPPED
    // dont do anything as you save data
    //  when loading data end status kidnap and start new world


    #if 0
    if( __a__ == __b__ ){
        ; // nothing to do all good
    }
    else if( __a__-1 == __b__ ) {
        // TODO
        cout << "obj[\"WorldsData\"][\"vec_world_ends\"].push_back( __ );\n";
        json ___y;
        ___y["stampNSec"] = last_node_timestamp;
        obj["WorldsData"]["vec_world_ends"].push_back( ___y );

    } else {
        cout << "Either __a__ has to be equal to __b__ or __a__ has to be 1 more than b. Currently this is not the case, this looks like something is wrong\n";
        exit(1);
    }


    if( __c__ == __d__ ){
        ; // nothing to do all good
    }
    else if( __c__ - 1 == __d__  ) {
        // TODO
        cout << "obj[\"KidnapTimestamps\"][\"kidnap_ends\"].push_back( __ );\n";
        json ___y;
        ___y["stampNSec"] = last_node_timestamp;
        obj["KidnapTimestamps"]["kidnap_ends"].push_back( ___y );

    } else {
        cout << "Either __c__ has to be equal to __d__ or __c__ has to be 1 more than b. Currently this is not the case, this looks like something is wrong\n";
        exit(1);
    }
    #endif

    //---
    // Save JSON
    cout << "obj[\"SolvedPoseGraph\"].size()=" << obj["SolvedPoseGraph"].size() << endl;
    RawFileIO::write_string( save_dir_path+"/solved_posegraph.json", obj.dump(4) );
    cout << TermColor::GREEN() <<  "DONE........    Composer::saveStateToDisk  ^^^^^^^^^^\n" << TermColor::RESET();
    return true;
}


bool Composer::loadStateFromDisk( string save_dir_path )
{
    cout << TermColor::GREEN();
    cout << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    cout << "^^^^^^^^^^    Composer::loadStateFromDisk  ^^^^^^^^^^\n";
    cout << "^^^^^^^^^^    DIR=" << save_dir_path ;
    cout << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    cout << TermColor::RESET();

    //---
    // Load JSON
    string json_fname = save_dir_path+"/solved_posegraph.json";
    cout << TermColor::GREEN() << "[Composer::loadStateFromDisk]Open file: " << json_fname << TermColor::RESET() <<  endl;
    std::ifstream json_fileptr(json_fname);
    if( !json_fileptr )
    {
        ROS_ERROR( "[Composer::loadStateFromDisk]Cannot load from previous state" );
        cout << TermColor::RED() << "[Composer::loadStateFromDisk]Fail to open" << json_fname << " file, perhaps it doesnt exist. Cannot load from previous state.\nEXIT(1)"<< endl;
        exit(1);
    }
    json json_obj;
    json_fileptr >> json_obj;
    cout << "[DataManager::loadStateFromDisk]Successfully opened file and loaded data "<< json_fname << endl;
    json_fileptr.close();


    //---
    // Load World Data into class Worlds
    bool status_w = manager->getWorldsPtr()->loadStateFromDisk( json_obj["WorldsData"] );
    if( status_w == false ) {
        cout << TermColor::RED() << "[Composer::loadStateFromDisk] manager->getWorldsPtr()->loadStateFromDisk returned false\nFATAL ERROR..." << TermColor::RESET() << endl;
        exit(1);
    }
    manager->getWorldsPtr()->print_summary();



    //---
    // Load kidnap timestamps into manager->kidnap_data
    bool status_k = manager->load_kidnap_data_from_json( json_obj["KidnapTimestamps"] );
    if( status_k == false )
    {
        cout << TermColor::RED() << "[Composer::loadStateFromDisk] manager->load_kidnap_data_from_json() returned false\nFATAL ERROR..." << TermColor::RESET() << endl;
        exit(1);
    }


    //---
    // Load Pose Graph
    bool status_pg = manager->load_solved_posegraph_data_from_json( json_obj );
    if( status_pg == false )
    {
        cout << TermColor::RED() << "[Composer::loadStateFromDisk] manager->load_solved_posegraph_data_from_json() returned false\nFATAL ERROR..." << TermColor::RESET() << endl;
        exit(1);
    }

    //---
    // Adjust variables in object slam (in PoseGraphSLAM), especially the solved_until
    bool status_slamsolver = slam->load_state(  ); //< this will build up a constant pose graph until this point.
    if( status_slamsolver == false )
    {
        cout << TermColor::RED() << "[Composer::loadStateFromDisk]slam->load_state(  ); returned false\nFATAL ERROR..." << TermColor::RESET() << endl;
        exit(1);
    }


    return true;

}
