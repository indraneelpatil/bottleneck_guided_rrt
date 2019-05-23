# Bottleneck Guided Rapidly Exploring Random Tree Star

The aim is to solve the problem of no 'one size fits all' heuristic for different motion planning problems and the inherent 'narrow passage' problem for uniform sampling based planners. 

The motion planner is implemented in C++ whereas the 3D CNN is implemented in Python using Tensorflow. The 3D CNN is a modified form of the [VoxNet](https://www.ri.cmu.edu/pub_files/2015/9/voxnet_maturana_scherer_iros15.pdf) architecture. 

### Modifications :

* Multi Input Single Output CNN 
  *1st Input : Voxelized Environment
  *2nd Input : Start and End points selected by user in Rviz*
  * Output : Relevant Bottleneck Points
* No non linear activation units in the last layer since ours is a prediction task (not classification)

### Training :

* Since data was limited for our prediction task, we have used transfer learning from a pre trained VoxNet trained on the Sydney Urban Objects Dataset.
* Our manually labelled dataset in the labelled dataset folder consists of 200 data samples collected using a 10% Goal Biased RRT* on different hand engineered environments.




