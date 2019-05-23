from nolearn.lasagne import BatchIterator
import numpy as np
import tensorflow as tf
import matplotlib
matplotlib.use('TkAgg', warn = False)
from matplotlib import pyplot
import os
import time

from pandas import DataFrame
from pandas.io.parsers import read_csv
from sklearn.utils import shuffle
from sklearn.cross_validation import train_test_split
import utils.data_helper as helper
import utils.visualization as viewer

os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
num_classes=14  #number of outputs
batch_size=32
num_epochs=8
voxel_size=32

DATA='/home/indraneel/tf-demo/VoxNet/npy_generated'
model_path = "/home/indraneel/tf-demo/VoxNet/checkpoint/model.ckpt"

# label dict for Sydney Urban Object Dataset, ref:http://www.acfr.usyd.edu.au/papers/SydneyUrbanObjectsDataset.shtml
SUOD_label_dictionary = {
    '4wd': 0, 'building': 1, 'bus': 2, 'car': 3, 'pedestrian': 4, 'pillar': 5, 'pole': 6,
    'traffic_lights': 7, 'traffic_sign': 8, 'tree': 9, 'truck': 10, 'trunk': 11, 'ute': 12, 'van': 13
}

def indices_to_one_hot(data, nb_classes):
    """Convert an iterable of indices to one-hot encoded labels."""
    targets = np.array(data).reshape(-1)
    return np.eye(nb_classes)[targets]

# Training data collector
voxels, labels = helper.load_data_from_npy(DATA, mode='training')
print("********************")
print(voxels.shape)
print(labels.shape)
print("********************")
#**************************************Visualize the input voxels*************
voxel_viz=voxels[2000]
#*********************************Print ground truth label of input voxels*****
print(labels[2000])
print(voxel_viz.shape)
viewer.plot3DVoxel(voxel_viz)
#voxels=tf.reshape(voxels,[-1,32,32,32,1])
#voxels=np.reshape(voxels,(-1,32,32,32,1))
#onehot_labels = tf.one_hot(indices=tf.cast(labels, tf.int32), depth=num_classes)
onehot_labels=indices_to_one_hot(labels,14)
print(voxels.shape)
print(onehot_labels.shape)
print(onehot_labels[0])

with tf.Session() as sess:
	model_saver = tf.train.import_meta_graph('/home/indraneel/tf-demo/VoxNet/checkpoint/model.ckpt.meta')
	ckpt = tf.train.get_checkpoint_state(os.path.dirname('/home/indraneel/tf-demo/VoxNet/checkpoint/checkpoint'))
	model_saver.restore(sess,ckpt.model_checkpoint_path)
	#model_saver.restore(sess,  root_location + model_name + '/checkpoint')
	#x=tf.get_collection('Placeholder')
	#output=tf.get_collection('CNN_test/add_5')
	#flag=tf.get_collection('Placeholder_2')
	graph = tf.get_default_graph()
	x=graph.get_tensor_by_name('Placeholder:0')
	output=graph.get_tensor_by_name('Relu_3:0')
	flag=graph.get_tensor_by_name('Placeholder_2:0')

	print("Model restored.")
	print('Initialized')
	#To find names of the placeholders to restore during prediction
	#graph = tf.get_default_graph()
	#for op in graph.get_operations():
	#	print(op.name)
	#p=[]
	#batch_iterator=BatchIterator(batch_size=128)
	#for x_batch,_ in batch_iterator(x_train):
	#	[p_batch]=sess.run([output],feed_dict={x:x_batch,flag:False})
	#	p.extend(p_batch)
	x_temp=voxels[2000].reshape(1,32,32,32,1)
	#**************************************Make Prediction using the trained network
	p=sess.run(output,feed_dict={x:x_temp,flag:False})
	#p=np.asarray(p)
	#p=p.reshape(-1)
	print(p)
	print(p.shape)
	#**************************************Plot the predictions on image side by side for comparison
	#ax = fig.add_subplot(1, 2, 2, xticks=[], yticks=[])
	#plot_sample(x_train[1], p, ax)

