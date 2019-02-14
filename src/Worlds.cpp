
#include "Worlds.h"



const Matrix4d Worlds::getPoseBetweenWorlds( int m, int n ) const
{
    assert( rel_pose_between_worlds__wb_T_wa.count( make_pair(m,n) ) > 0 );
    return rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ];
}



bool Worlds::setPoseBetweenWorlds( int m, int n, const Matrix4d m_T_n )
{
    rel_pose_between_worlds__wb_T_wa[ make_pair(m,n) ] = m_T_n;
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

    }

    cout << "\\---- END Relative Poses Between Worlds----/\n";
}
