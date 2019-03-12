# Node for pose graph optimization.

This node actually builds up the graph of poses along with odometry edges and loop closure edges.
To get a theoritical understanding of the process, I suggest to look at this : [HERE](https://kusemanohar.wordpress.com/2017/04/29/howto-pose-graph-bundle-adjustment/)

Defines 2 classes
### NodeDataManager:
This class holds all the data. Including node data and
critical edge data. Later on can have more data if need be.
Crititcal data include: i) VIO poses at each keyframes. ii) Edge poses.

### PoseGraphSLAM:
This class holds the pointer to an object of class NodeDataManager.
All the functions related to ceres-solver here. It is intended to be run in a separate thread.

### main (keyframe_pose_graph_slam_node.cpp)
This contains the main. It calls a visualization publisher in 1 thread. In another thread PoseGraphSLAM->optimize6DOF() is called.
The main thread handles the call back. Subscribes to `camera_pose` and `colocation_chatter`. Publishes
a) visualization marker and b) path (only contains last 5 posestamped)

The visualization marker is a `LINE_STRIP` at all pose. The LINE_STRIP  will be pink in color
to denote the poses have been processed by ceres::Solve(). Green colored lines means those poses
are purely odometry and have not yet been ceres::Solved()d.


## Dependencies
- ceres-solver
- Eigen3
- OpenCV 3 (only used for visualization)
- [Cerebro pkg](https://github.com/mpkuse/cerebro) (for definations of LoopEdgeMsg)

## Author
Manohar Kuse <mpkuse@connect.ust.hk>
