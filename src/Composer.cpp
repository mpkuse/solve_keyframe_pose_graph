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
            __Composer__pose_assember_thread_posedebug(
            cout << "i=" << i << " world#" << world_id << endl;
            )

            if( i>=0 && i<= ____solvedUntil )
            {
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
                int last_idx=-1;
                Matrix4d w_TM_i;

                if( ____solvedUntil == 0 ) {
                    w_TM_i = manager->getNodePose( i );
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



// #define __Composer_bf_traj_publish_thread( msg ) msg;
#define __Composer_bf_traj_publish_thread( msg ) ;
void Composer::bf_traj_publish_thread( int looprate )
{
    //--- Options
    const int options_line_color_style = 10; // should be either 10 or 12. 10:color by WorldID; 12:  //color by world setID
    const float options_linewidth_multiplier  = 3; // thickness of the line

    //---

    //          Note: global_jmb's keys are `worldIDs` and global_lmb's values are `vector<Matrix4d>& w_T_ci`
    cout << TermColor::GREEN() << "start `Composer::bf_traj_publish_thread` @ " << looprate << " hz" << TermColor::RESET() << endl;
    assert( looprate > 0 && looprate < 50 );
    ros::Rate rate(looprate);

    static thread_local std::mt19937 generator;
    std::uniform_int_distribution<int> distribution(0,100);

    while( b_bf_traj_publish )
    {
        rate.sleep();
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

            bool publish_all = (distribution(generator) < 5)?true:false;

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

    }

    cout << TermColor::RED() << "Finished `Composer::bf_traj_publish_thread`\n" << TermColor::RESET() << endl;
}



void Composer::cam_visual_publish_thread( int looprate )
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



// #define __Composer__loopedge_publish_thread(msg) msg;
#define __Composer__loopedge_publish_thread(msg) ;
void Composer::loopedge_publish_thread( int looprate )
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
void Composer::disjointset_statusimage_publish_thread( int looprate )
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
