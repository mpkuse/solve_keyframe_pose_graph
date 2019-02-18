
#include "Worlds.h"



const Matrix4d Worlds::getPoseBetweenWorlds( int m, int n ) const
{
    std::lock_guard<std::mutex> lk(mutex_world);

    //TODO: the relative pose can be found if both world-m and world-n are in same set.
    // So, even if this doesn't exist in map, it can be infered.

    assert( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 );
    Matrix4d __tmp = rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ];
    return  __tmp;


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
    disjoint_set_debug += "union_sets( " + std::to_string(m) + "," + std::to_string(n) + ")\n";
}

bool Worlds::is_exist( int m, int n ) const
{
    std::lock_guard<std::mutex> lk(mutex_world);

    if( m<0 || n<0 )
        return false;

    if( m==n )
        return true;

    if( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 ) {
        return true;
    }
    else {
        // cout << TermColor::RED() << "[Worlds::is_exist] cannot find rel pose between m=" << m << " and n=" << n << ".\n" << TermColor::RESET();
        // cout << "TODO need to implement inference if they belong to same set, m belongs to set " << find_setID_of_world_i(m) << " and n belongs to set " << find_setID_of_world_i( n ) << endl;
        return false;
    }
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

void Worlds::world_starts( ros::Time _t )
{
    std::lock_guard<std::mutex> lk(mutex_world);

    vec_world_starts.push_back( _t );

    // Add a new element in disjoiunt set.
    disjoint_set.add_element( vec_world_starts.size() -1, true );
    disjoint_set_debug += "add_element( " + std::to_string( vec_world_starts.size() -1 ) + ")\n";
}

void Worlds::world_ends( ros::Time _t )
{
    std::lock_guard<std::mutex> lk(mutex_world);

    vec_world_ends.push_back( _t );
}

int Worlds::find_setID_of_world_i( int i ) const //< returns the setid of world i
{
    std::lock_guard<std::mutex> lk(mutex_world);

    return disjoint_set.find_set( i );
}

int Worlds::n_worlds() const //< number of worlds
{
    std::lock_guard<std::mutex> lk(mutex_world);

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

void Worlds::print_summary() const
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
        cout << " info=" << rel_pose_between_worlds__wb_T_wa___info_string[ it->first ] ;
        cout << endl;

    }
    cout << "\t\t\\---- END Relative Poses Between Worlds----/\n";

    cout << "\n\t\t----Info from DisjointSet ----\n";
    cout << "\t\telement count: " << disjoint_set.element_count() << std::endl;
    cout << "\t\tset_count: " << disjoint_set.set_count() << endl;
    for( int i=0 ; i<disjoint_set.element_count() ; i++ ) {
        cout << "\t\tworld#" << i << "  is in set=" << disjoint_set.find_set( i ) ;
        // cout << "  value=" << disjoint_set.value_of( i );
        cout << endl;
    }

    cout << "\n\t\t```all_ops performed:\n";
    cout << disjoint_set_debug;
    cout << "\n\t\t```\n";

    cout << "\t\t\\---- END Info from DisjointSet----/\n";


}
