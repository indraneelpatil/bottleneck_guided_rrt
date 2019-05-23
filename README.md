# VoxNet: A 3D Convolutional Neural Network for Real-Time Object Recognition

　An on going TF implementation on VoxNet to deal with 3D LiDAR pointcloud segmentation classification, refer to [paper](https://www.ri.cmu.edu/pub_files/2015/9/voxnet_maturana_scherer_iros15.pdf).

```bibtex
@inproceedings{Maturana2015VoxNet,
  title={VoxNet: A 3D Convolutional Neural Network for real-time object recognition},
  author={Maturana, Daniel and Scherer, Sebastian},
  booktitle={Ieee/rsj International Conference on Intelligent Robots and Systems},
  pages={922-928},
  year={2015},
}
```

<p align="center">
   <img src="./readme/The VoxNet Architecture.png" width="420" alt="" />
</p>

+ **Input Layer**. This layer accepts a fixed-size grid of `I×J×K` (I=J=K=32) voxels, each value for each grid cell is updated depending on the occupancy model, resulted in the (−1, 1) range.
+ **Convolutional Layers** `Conv(f, d, s)`.
   + These layers accept four dimensional input volumes in which three of the dimensions are spatial, and the fourth contains the feature maps.
   + The layer creates `f` feature maps by convolving the input with `f` learned filters of shape `d × d × d × f'`, where `d` are the spatial dimensions and f' is the number of input feature maps. <br/> **==>** **conv3d(depth=d, height=d, width=d, in_channels=f', out_channels=f)**
   + Convolution can also be applied at a **spatial stride** `s`.
   + Output shape: `(I/J/K - d + 2*padding)/s + 1`
   + The output is passed through a leaky rectified nonlinearity unit (**Leaky ReLU**) with parameter 0.1. (激活函数)
+ **Pooling Layers** `Pool(m)`.
   + These layers downsample the input volume by a factor of by `m` along the spatial dimensions by replacing each `m × m × m` **non-overlapping block** of voxels with their maximum. <br/> **≈** **conv3d(depth=m, height=m, width=m, in_channels=f, out_channels=m) & s=m**
+ **Fully Connected Layer** `FC(n)`.
   + Fully connected layers have `n` output neurons. The output of each neuron is **a learned linear combination** of all the outputs from the previous layer, passed through a nonlinearity.
+ **Output Layer**.
   + **ReLUs** save for the final output layer, where the number of outputs corresponds to the number of class labels `K` and a **softmax nonlinearity** is used to provide a probabilistic output.
+ **VoxNet**: `Conv(32, 5, 2)`→`Conv(32, 3, 1)`→`Pool(2)`→`FC(128)`→`FC(K)`

## Dataset
+ [Sydney Urban Object Dataset, short for SUOD](http://www.acfr.usyd.edu.au/papers/SydneyUrbanObjectsDataset.shtml)
+ Other LiDAR PointCloud Dataset(not yet support though :D): <br/>
   [Stanford Track Collection](http://cs.stanford.edu/people/teichman/stc/)  
   [**KITTI Object Recognition**](http://www.cvlibs.net/datasets/kitti/eval_object.php)  
   [Semantic 3D](http://www.semantic3d.net/view_dbase.php?chl=2) 


## Requirement
　Implemented and tested on Ubuntu 16.04 with Python 3.5 and Tensorflow 1.3.0.
1. Clone this repo
   ```bash
   $ git clone https://github.com/Durant35/VoxNet --recurse-submodules
   ```
   We name the root directory as `$ROOT` and if you forget to clone the [python-pcl](https://github.com/Durant35/python-pcl) submodule:
   ```bash
   $ git submodule update --init --recursive
   ```
   
2. Setup virtual environment with all requirements
   ```bash
   $ mkvirtualenv --no-site-packages -p /usr/bin/python3.5 py3-1.3.0
   $ cd $ROOT
   $ workon py3-1.3.0
   (py3-1.3.0) $ pip3 install -r requirements.txt
   ```
   
3. [option] [python-pcl](https://github.com/Durant35/python-pcl), or you can comment those pcl codes.
   ```bash
   $ cd $ROOT
   $ workon py3-1.3.0
   (py3-1.3.0) $ pip3 install Cython
   (py3-1.3.0) $ cd 3rdparty/python-pcl
   (py3-1.3.0) $ python setup.py build_ext -i
   (py3-1.3.0) $ python setup.py install
   (py3-1.3.0) $ rm -rf *
   ```
   
## Data pre-process
　Generate `npy_generated/training/*.py` from **SUDO** fold 1-3, `npy_generated/testing/*.py` from fold 4.
   ```bash
   $ cd $ROOT
   $ workon py3-1.3.0
   (py3-1.3.0) $ python ./src/preprocess.py -h
   usage: preprocess.py [-h] [--dataset_dir DATASET_DIR] [--fold FOLD]
                        [--viz [VIZ]] [--noviz] [--pcd [PCD]] [--nopcd]
                        [--npy_dir NPY_DIR] [--clear_cache [CLEAR_CACHE]]
                        [--noclear_cache] [--type TYPE]
   
   optional arguments:
     -h, --help            show this help message and exit
     --dataset_dir DATASET_DIR
                           directory that stores the Sydney Urban Object Dataset,
                           short for SUOD.
     --fold FOLD           which fold, 0..3, for SUOD.
     --viz [VIZ]           visualize preprocess voxelization.
     --noviz
     --pcd [PCD]           save object point cloud as pcd.
     --nopcd
     --npy_dir NPY_DIR     directory to stores the SUOD preprocess results,
                           including occupancy grid and label.
     --clear_cache [CLEAR_CACHE]
                           clear previous generated preprocess results.
     --noclear_cache
     --type TYPE           type of SUOD preprocess results, training set or
                           testing set.
   # prepare training set & testing set
   (py3-1.3.0) $ python ./src/preprocess.py --clear_cache
   (py3-1.3.0) $ python ./src/preprocess.py --fold 1
   (py3-1.3.0) $ python ./src/preprocess.py --fold 2
   (py3-1.3.0) $ python ./src/preprocess.py --fold 3 --type testing
   ```

## Training/Validation
+ Run at once.
   ```bash
   $ workon py3-1.3.0
   (py3-1.3.0) $ ./scripts/train.sh
   ...
   Start training...
   ...
   INFO:tensorflow:loss = 0.004891032, step = 208 (7.759 sec)
   INFO:tensorflow:Saving checkpoints for 214 into ./logs/model.ckpt.
   INFO:tensorflow:Loss for final step: 0.007142953.
   Finished training.
   Start testing...
   INFO:tensorflow:Starting evaluation at 2018-04-26-03:29:16
   2018-04-26 11:29:16.387119: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1045] Creating TensorFlow device (/gpu:0) -> (device: 0, name: GeForce 940MX, pci bus id: 0000:01:00.0)
   INFO:tensorflow:Restoring parameters from ./logs/model.ckpt-214
   INFO:tensorflow:Finished evaluation at 2018-04-26-03:29:16
   INFO:tensorflow:Saving dict for global step 214: accuracy = 0.64666665, global_step = 214, loss = 2.4514768
   Finished testing.
   You can use Tensorboard to visualize the results by command 'tensorboard --logdir=./logs'.
   ```
   
+ Run after pre-process `npy_generated/training/*.npy`
   ```bash
   $ cd $ROOT
   $ workon py3-1.3.0
   (py3-1.3.0) $ python ./src/train.py -h
   usage: train.py [-h] [--log_dir LOG_DIR] [--npy_dir NPY_DIR]
                   [--clear_log [CLEAR_LOG]] [--noclear_log]
                   [--num_epochs NUM_EPOCHS] [--batch_size BATCH_SIZE]
   
   optional arguments:
     -h, --help            show this help message and exit
     --log_dir LOG_DIR     Directory for training logs, including training
                           summaries as well as training model checkpoint.
     --npy_dir NPY_DIR     directory to the preprocess training dataset.
     --clear_log [CLEAR_LOG]
                           force to clear old logs if exist.
     --noclear_log
     --num_epochs NUM_EPOCHS
                           The numbers of epochs for training, train over the
                           dataset about 8 times.
     --batch_size BATCH_SIZE
                           The numbers of training examples present in a single
                           batch for every training.
   (py3-1.3.0) $ python ./src/train.py  --clear_log
   ```
   
## Testing
```bash
$ cd $ROOT
$ workon py3-1.3.0
(py3-1.3.0) $ python ./src/eval.py -h
usage: eval.py [-h] [--model_dir MODEL_DIR] [--npy_dir NPY_DIR]

optional arguments:
  -h, --help            show this help message and exit
  --model_dir MODEL_DIR
                        directory for training model checkpoint.
  --npy_dir NPY_DIR     directory to the preprocess training dataset.
# run on default configs.
(py3-1.3.0) $ python ./src/eval.py
...
Predicted: trunk, Ground Truth: trunk
Top 3 labels: trunk traffic_sign traffic_lights...
Predicted: pedestrian, Ground Truth: pedestrian
Top 3 labels: pedestrian traffic_sign car...
Predicted: pedestrian, Ground Truth: pedestrian
Top 3 labels: pedestrian traffic_sign traffic_lights...
Predicted: traffic_lights, Ground Truth: traffic_lights
Top 3 labels: traffic_lights trunk building...
INFO:tensorflow:Starting evaluation at 2018-04-28-10:09:34
2018-04-28 18:09:34.279983: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1045] Creating TensorFlow device (/gpu:0) -> (device: 0, name: GeForce 940MX, pci bus id: 0000:01:00.0)
INFO:tensorflow:Restoring parameters from ./logs/model.ckpt-1381
2018-04-28 18:09:36.986620: W tensorflow/core/framework/op_kernel.cc:1192] Out of range: FIFOQueue '_1_enqueue_input/fifo_queue' is closed and has insufficient elements (requested 128, current size 0)
	 [[Node: fifo_queue_DequeueUpTo = QueueDequeueUpToV2[component_types=[DT_INT64, DT_FLOAT, DT_INT64], timeout_ms=-1, _device="/job:localhost/replica:0/task:0/cpu:0"](enqueue_input/fifo_queue, fifo_queue_DequeueUpTo/n)]]
INFO:tensorflow:Finished evaluation at 2018-04-28-10:09:37
INFO:tensorflow:Saving dict for global step 1381: accuracy = 0.64672804, global_step = 1381, loss = 2.816635
Finished testing.
```