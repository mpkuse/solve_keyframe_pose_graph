
#include "Worlds.h"


// m and n are worldIDs. A rel pose between two world will exist if they are in same set.
const Matrix4d Worlds::getPoseBetweenWorlds( int m, int n ) const
{

    #if 0
    // old simplikstic code
    //TODO: the relative pose can be found if both world-m and world-n are in same set.
    // So, even if this doesn't exist in map, it can be infered.
    std::lock_guard<std::mutex> lk(mutex_world);
    assert( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 );
    Matrix4d __tmp = rel_pose_between_worlds__wb_T_wa.at( make_pair(m,n) );
    return  __tmp;
    #endif

    if( m==n )
        return Matrix4d::Identity();


    if( is_exist(m,n) == false ) {
        cout << TermColor::RED() << "[Worlds::getPoseBetweenWorlds] you requested a relative pose between world#" << m << " and world#" << n << endl;
        cout << "is_exist(m,n) has returned false in this case indicating that both of them are in different sets or non-exisitant sets. You should have called is_exist() before calling this function.\n";
        cout << "FATAL ERROR....QUITIING...." << TermColor::RESET() << endl;
        exit(5);
    }

    // It is extablished that relative can definately be found...(since both m and n belong to same set)
    // case-a: Simple : see if the requested entry exists in the map.
    // case-b: Tricky : the requested entry is not in the map, but can be infered from the
    //                  exisiting entries in the map. Compute the requested pose from exisiting entry
    //                  and add a new entry into the map.


    // case-a: Simple
    {
    std::lock_guard<std::mutex> lk(mutex_world);
    if( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0  ) {
        Matrix4d __tmp = rel_pose_between_worlds__wb_T_wa.at( make_pair(m,n) );
        return  __tmp;
    }

    // also see if n,m can be found in the map. if so return its inverse.
    if( rel_pose_between_worlds__wb_T_wa.count( make_pair(n,m) ) > 0  ) {
        Matrix4d __tmp = rel_pose_between_worlds__wb_T_wa.at( make_pair(n,m) ).inverse();
        return  __tmp;
    }
    }


    // case-b: Tricky
    #define ___WORLDS_BFS_( msg ) msg;
    cout << TermColor::RED() << "[[Worlds::getPoseBetweenWorlds]] tricky-case.\n";
    int setID;
    Matrix4d ans = Matrix4d::Identity();
    int ans_m, ans_n;
    {
    std::lock_guard<std::mutex> lk(mutex_world);
    setID = disjoint_set.find_set( m ); //disjoint_set.find_set( n ) will give same result.
    cout << "You request relative pose between world#" << m << " and world#" << n << "  setID=" << setID << endl;


    // :::TODO A: Compute the pose. becareful of the locks. calling a function which takes lock will stall the code.
    //      Solution
    //          1. filter all elements which belong to setID. loop through the map
    //          2. form a undirected graph
    //          3. keep m as root and do a breadth-first-search until n is found.
    //          4. From the chain aka path from m to n, the pose can be infered by chaining the relative poses from the map

    //------Step-1&2: loop through map.
    cout << ">>> init graph with " <<n_worlds__nolock() << " nodes\n";
    MyDirectionalGraph graph( n_worlds__nolock() );
    for( auto it= rel_pose_between_worlds__wb_T_wa.begin() ;
        it!= rel_pose_between_worlds__wb_T_wa.end() ; it++ )
    {
        int _m = std::get<0>( it->first );
        int _n = std::get<1>( it->first );
        // if ( _m is in setID and _n is in setID ) then skip this 'it'
        if( !( disjoint_set.find_set(_m) == setID && disjoint_set.find_set(_n) == setID ) )
            continue;

        graph.add_edge( _m, _n );
        graph.add_edge( _n, _m );
        cout << ">>> add_edge("   << _m << "," << _n << ")" ;
        cout << "\tadd_edge(" << _n << "," << _m << ")" << endl;

    }


    //---------Step-3: BFS
    cout << ">>> graph.BFS("<< n << ")\n";
    graph.BFS( n );
    graph.print_intermediate_debug();
    vector<int> path;
    cout << ">>> graph.get_path_from( " << m << " )\n";
    graph.get_path_from( m, path );
    cout<< "path.size=" << path.size() << "\t";    for( int h=0; h<path.size() ; h++ ) {cout << path[h] << ",";} cout << endl;


    //-----------Step-4: \PI path[i]__T__path[i+1]
    cout << ">>> ans=" ;
    ans_m = path[0];
    ans_n = path[ path.size() - 1 ];
    for( int h=0 ; h<path.size()-1 ; h++ )
    {
        // cout << "w" << path[h] << "__T__w" << path[h+1] << " x ";

        // Matrix4d __tmp = rel_pose_between_worlds__wb_T_wa.at( make_pair( path[h], path[h+1] ) );
        Matrix4d h_T_hp1;
        if( rel_pose_between_worlds__wb_T_wa.count( make_pair( path[h], path[h+1] ) ) > 0 ) {
            h_T_hp1 = rel_pose_between_worlds__wb_T_wa.at( make_pair( path[h], path[h+1] ) );
            cout << "w" << path[h] << "__T__w" << path[h+1] << " x ";
        }
        else{
            if( rel_pose_between_worlds__wb_T_wa.count( make_pair( path[h+1], path[h] ) ) > 0 ) {
                h_T_hp1 = rel_pose_between_worlds__wb_T_wa.at( make_pair( path[h+1], path[h] ) ).inverse();
                cout << "inv( w" << path[h+1] << "__T__w" << path[h] << ") x ";
            }
            else {
                cout << "[Worlds::getPoseBetweenWorlds/tricky-case] FATAL ERROR EXIT(582392)\n";
                exit( 2 );
            }
        }

        ans *= h_T_hp1;

    }
    cout << endl;

    cout << ">>> ans=" << PoseManipUtils::prettyprintMatrix4d( ans ) << endl;


    }
    // :::TODO B:  setPoseBetweenWorlds(). need to deal with locking.
    setPoseBetweenWorlds( ans_m, ans_n, ans, "pose set by inference with BFS" );



    print_summary();
    cout << TermColor::RESET() << endl;
    // exit( 3 );





}





bool Worlds::setPoseBetweenWorlds( int m, int n, const Matrix4d m_T_n, const string info_string )
{
    std::lock_guard<std::mutex> lk(mutex_world);

    rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ] = m_T_n;

    rel_pose_between_worlds__wb_T_wa___info_string[ make_pair(m,n) ] +=  ";" + info_string;

    // union_sets()
    assert( disjoint_set.exist( m ) && disjoint_set.exist( n ) && "Either of m of n doesn't seem to exist in the disjoint set. m="+to_string(m)+" n="+to_string(n) );

    // ideally, disjoint_set.union_sets( m, n ), but doing the min max trick to retain the id of earliest sample.
    disjoint_set.union_sets( max(m,n), min(m,n) );
    disjoint_set_debug += "\t\t\tunion_sets( " + std::to_string(m) + "," + std::to_string(n) + ")\n";
}

// m and n are worldIDs. A rel pose between two world will exist if they are in same set.
// this will return true for (n,m) as well.
bool Worlds::is_exist( int m, int n ) const
{

    if( m<0 || n<0 )
        return false;

    if( m==n )
        return true;

    {
    std::lock_guard<std::mutex> lk(mutex_world);
    int setid_of_m = disjoint_set.find_set( m ); //find_setID_of_world_i(m);
    int setid_of_n = disjoint_set.find_set( n ); //find_setID_of_world_i(n);
    if(  setid_of_m >= 0 && setid_of_n >=0 && setid_of_m == setid_of_n )
        return true;
    else
        return false;
    }

    // TODO : removal of following code. old code.
    //
    // if( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 ) {
    //     return true;
    // }
    // else {
    //     // cout << TermColor::RED() << "[Worlds::is_exist] cannot find rel pose between m=" << m << " and n=" << n << ".\n" << TermColor::RESET();
    //     // cout << "TODO need to implement inference if they belong to same set, m belongs to set " << find_setID_of_world_i(m) << " and n belongs to set " << find_setID_of_world_i( n ) << endl;
    //     return false;
    // }
}

void Worlds::getAllKeys( vector<std::pair<int,int> >& all_keys ) const
{
    std::lock_guard<std::mutex> lk(mutex_world);

    // This function is not tested
    all_keys.clear();
    for( auto it= rel_pose_between_worlds__wb_T_wa.begin() ;
        it!= rel_pose_between_worlds__wb_T_wa.end() ; it++ )
    {
        all_keys.push_back( it->first );
    }
}

void Worlds::getWorld2SetIDMap( std::map<int,int>& mipmap ) const
{
    mipmap.clear();
    for( int ww=0; ww<n_worlds() ; ww++ ) {
        mipmap[ww] = find_setID_of_world_i( ww );
    }
}

void Worlds::world_starts( ros::Time _t )
{
    std::lock_guard<std::mutex> lk(mutex_world);

    vec_world_starts.push_back( _t );

    // Add a new element in disjoiunt set.
    disjoint_set.add_element( vec_world_starts.size() -1, true );
    disjoint_set_debug += "\t\t\tadd_element( " + std::to_string( vec_world_starts.size() -1 ) + ")\n";
}

void Worlds::world_ends( ros::Time _t )
{
    std::lock_guard<std::mutex> lk(mutex_world);

    vec_world_ends.push_back( _t );
}

int Worlds::find_setID_of_world_i( int i ) const //< returns the setid of world i
{
    std::lock_guard<std::mutex> lk(mutex_world);
    if( disjoint_set.exists(i) ) // do a find query only when you are sure that this element exisits in set.
        return disjoint_set.find_set( i );
    return -1;
}

int Worlds::n_worlds() const //< number of worlds
{
    std::lock_guard<std::mutex> lk(mutex_world);

    return disjoint_set.element_count();
}


int Worlds::n_worlds__nolock() const //< number of worlds
{
    return disjoint_set.element_count();
}


int Worlds::n_sets() const  //< number of sets
{
    std::lock_guard<std::mutex> lk(mutex_world);

    return disjoint_set.set_count();
}

const string Worlds::disjoint_set_status() const
{
    std::lock_guard<std::mutex> lk(mutex_world);

    std::map<int,string> ff;
    string output_string = string();

    output_string += string("element_count=")+to_string( disjoint_set.element_count()  );
    output_string += string("   set_count=")+to_string( disjoint_set.set_count()  );
    output_string += ";";

    // loop through all elements
    for( int i=0 ; i<disjoint_set.element_count() ; i++ ) {
        int setID = disjoint_set.find_set( i ) ;
        output_string += "world#"+ to_string(i) + " is in setID=" + to_string(setID) + ";" ;

        if( ff.count(setID) > 0  )
            ff[ setID ] += "," + to_string( i );
        else
            ff[ setID ] = to_string( i );
    }

    output_string += ";";
    for( auto it = ff.begin() ; it!=ff.end() ; it++ )
    {
        output_string += "set#" + to_string(it->first) + " contains worlds: " + it->second + ";";
    }

    return output_string;

}

void Worlds::disjoint_set_status_image(cv::Mat& im_disp, bool enable_bubbles, bool enable_text ) const
{


    im_disp = cv::Mat::zeros(cv::Size(400,100), CV_8UC3);

    if( enable_bubbles ) {
        int uu = n_worlds(); //disjoint_set.element_count();
        // circles
        cv::putText( im_disp, "Worlds:", cv::Point(5, 10), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(255,255,255), 1.5 );
        cv::putText( im_disp, "SetIDs of Worlds:", cv::Point(5, 65), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(255,255,255), 1.5 );
        for( int i=0 ; i<uu; i++ ) {
            // World Colors
            cv::Scalar color = FalseColors::randomColor( i );
            // cout << color << endl;
            cv::Point pt = cv::Point(20*i+15, 30);
            cv::circle( im_disp, pt, 10, color, -1 );
            cv::putText( im_disp, to_string(i), pt, cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(255,255,255), 1.5 );


            // Set Colors
            int setId =  find_setID_of_world_i( i );
            color = FalseColors::randomColor( setId );
            pt = cv::Point(20*i+15, 85);
            cv::circle( im_disp, pt, 10, color, -1 );
            cv::putText( im_disp, to_string(setId), pt, cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(255,255,255), 1.5 );


        }
    }

    if( enable_text ) {
        string info_msg = disjoint_set_status();
        FalseColors::append_status_image( im_disp, info_msg );
    }



}

void Worlds::print_summary(int verbosity ) const
{
    std::lock_guard<std::mutex> lk(mutex_world);

    cout << "\t\t----Relative Poses Between Worlds----\n";
    int r=0;
    for( auto it= rel_pose_between_worlds__wb_T_wa.begin() ;
        it!= rel_pose_between_worlds__wb_T_wa.end() ; it++, r++ )
    {
        int m = it->first.first;
        int n = it->first.second;
        Matrix4d m_T_n = it->second;
        cout << "\t\t" << r << ")" << m<< "_T_" << n << " = " << PoseManipUtils::prettyprintMatrix4d( m_T_n );
        cout << " info=" << rel_pose_between_worlds__wb_T_wa___info_string.at( it->first ) ;
        cout << endl;

    }
    cout << "\t\t\\---- END Relative Poses Between Worlds----/\n";

    cout << "\n\t\t----Info from DisjointSet ----\n";
    cout << "\t\telement count: " << disjoint_set.element_count() ;
    cout << "\tset_count: " << disjoint_set.set_count() << endl;
    for( int i=0 ; i<disjoint_set.element_count() ; i++ ) {
        cout << "\t\tworld#" << i << "  is in set=" << disjoint_set.find_set( i ) ;
        // cout << "  value=" << disjoint_set.value_of( i );
        cout << endl;
    }

    if( verbosity > 0 ) {
    cout << "\n\t\t```all_ops performed:\n";
    cout << disjoint_set_debug;
    cout << "\n\t\t```\n";
    }

    cout << "\t\t\\---- END Info from DisjointSet----/\n";


}
