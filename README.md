#Node for pose graph optimization.

Defines 2 classes
### NodeDataManager:
This class holds all the data. Including node data and
critical edge data. Later on can have more data if need be.
Crititcal data include: i) VIO poses at each keyframes. ii) Edge poses.

### PoseGraphSLAM:
This class holds the pointer to an object of class NodeDataManager.
All the functions related to ceres-solver here. It is intended to be run in a separate thread.
