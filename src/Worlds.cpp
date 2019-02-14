
#include "Worlds.h"



const Matrix4d Worlds::getPoseBetweenWorlds( int m, int n ) const
{
    assert( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 );
    return rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ];
}



bool Worlds::setPoseBetweenWorlds( int m, int n, const Matrix4d m_T_n, const string info_string )
{
    rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ] = m_T_n;

    rel_pose_between_worlds__wb_T_wa___info_string[ make_pair(m,n) ] +=  ";" + info_string;

    // union_sets()
    assert( disjoint_set.exist( m ) && disjoint_set.exist( n ) && "Either of m of n doesn't seem to exist in the disjoint set. m="+to_string(m)+" n="+to_string(n) );
    disjoint_set.union_sets( m, n );
    disjoint_set_debug += "union_sets( " + std::to_string(m) + "," + std::to_string(n) + ")\n";
}

bool Worlds::is_exist( int m, int n ) const
{
    if( m<0 || n<0 )
        return false;

    if( m==n )
        return true;

    if( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 )
        return true;
    else
        return false;
}

void Worlds::getAllKeys( vector<std::pair<int,int> >& all_keys ) const
{
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
    vec_world_starts.push_back( _t );

    // Add a new element in disjoiunt set.
    disjoint_set.add_element( vec_world_starts.size() -1, true );
    disjoint_set_debug += "add_element( " + std::to_string( vec_world_starts.size() -1 ) + ")\n";
}

void Worlds::world_ends( ros::Time _t )
{
    vec_world_ends.push_back( _t );
}


void Worlds::print_summary()
{
    cout << "----Relative Poses Between Worlds----\n";
    int r=0;
    for( auto it= rel_pose_between_worlds__wb_T_wa.begin() ;
        it!= rel_pose_between_worlds__wb_T_wa.end() ; it++, r++ )
    {
        int m = it->first.first;
        int n = it->first.second;
        Matrix4d m_T_n = it->second;
        cout <<r << ")" << m<< "_T_" << n << " = " << PoseManipUtils::prettyprintMatrix4d( m_T_n );
        cout << " info=" << rel_pose_between_worlds__wb_T_wa___info_string[ it->first ] ;
        cout << endl;

    }
    cout << "\\---- END Relative Poses Between Worlds----/\n";

    cout << "\n----Info from DisjointSet ----\n";
    cout << "element count: " << disjoint_set.element_count() << std::endl;
    cout << "set_count: " << disjoint_set.set_count() << endl;
    for( int i=0 ; i<disjoint_set.element_count() ; i++ ) {
        cout << "world#" << i << "  is in set=" << disjoint_set.find_set( i ) ;
        // cout << "  value=" << disjoint_set.value_of( i );
        cout << endl;
    }

    cout << "\n```\n";
    cout << disjoint_set_debug; 
    cout << "\n```\n";

    cout << "\\---- END Info from DisjointSet----/\n";


}
