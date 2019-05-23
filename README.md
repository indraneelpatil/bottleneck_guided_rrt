# Bottleneck Guided Rapidly Exploring Random Tree Star

<div align='right'>
  <img src='images/Flow chart 2(1).png'>
</div>


The aim is to solve the problem of no 'one size fits all' heuristic for different motion planning problems and the inherent 'narrow passage' problem for uniform sampling based planners. 

The motion planner is implemented in C++ whereas the 3D CNN is implemented in Python using Tensorflow. The 3D CNN is a modified form of the [VoxNet](https://www.ri.cmu.edu/pub_files/2015/9/voxnet_maturana_scherer_iros15.pdf) architecture. 

### Modifications :

* Multi Input Single Output CNN 
  * 1st Input : Voxelized Environment
  * 2nd Input : Start and End points selected by user in Rviz
  * Output : Relevant Bottleneck Points
* No non linear activation units in the last layer since ours is a prediction task (not classification)

### Training :

* Since data was limited for our prediction task, we have used transfer learning from a pre trained VoxNet trained on the Sydney Urban Objects Dataset.
* Our manually labelled dataset in the labelled dataset folder consists of 200 data samples collected using a 10% Goal Biased RRT* on different hand engineered environments.

## RRT* Implementation Details 
* [FCL](https://github.com/flexible-collision-library/fcl) : Collision Checking with environment
* [Octomap](https://github.com/OctoMap/octomap) : Data Structure for the 3D Environment
* [Mersenne Twister PRNG](https://github.com/cslarsen/mersenne-twister) : Sampling Random Points in Workspace of Manipulator
* [OpenRAVE](https://github.com/rdiankov/openrave) :Kinematic Reachability Module used to find sampling domain of the planner.
* [libNABO](https://github.com/ethz-asl/libnabo) :Library for nearest neighbour searches to find multiple parent candidates
* [H5Easy](https://github.com/stevenwalton/H5Easy): To convert hdf5 file created by openrave using C++ vectors

## References 
* [Execution Extended RRT](https://link.springer.com/content/pdf/10.1007/978-3-540-45135-8_23.pdf)
* [Synchronized Greedy Biased RRT](https://link.springer.com/content/pdf/10.1007/s12555-011-0417-7.pdf)




