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

os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

num_keypoints=9 #number of outputs of the network
batch_size=5  #32
num_epochs=100
voxel_size=32

DATA='/home/indraneel/tf-demo/VoxNet/rrt_data'
model_path = "/home/indraneel/tf-demo/VoxNet/rrt_checkpoint/model.ckpt"

###########################################################

########## Insert training and testing data here#############

##### voxels (m,32,32,32,1)
##### labels (m,9)

voxels,labels,endpoints=helper.load_labelled_data()
voxels=np.reshape(voxels,(-1,32,32,32,1))
print(voxels.shape)
labels=np.reshape(labels,(202,-1))
print(labels.shape)
print(endpoints.shape)

#We shuffle data using seed to get same result every time
voxels, labels = shuffle(voxels, labels, random_state=42)  # shuffle train data

print(voxels.shape)
print(labels.shape)

input("Training data Sucessfully loaded....Press Enter to continue...")

#############################################################

def conv_relu(input,kernel_size,depth,stride,scope_name):
	with tf.variable_scope(scope_name, reuse=tf.AUTO_REUSE) as scope:
		kernel=tf.get_variable('kernel',shape=[kernel_size,kernel_size,kernel_size,input.get_shape()[4],depth],initializer=tf.contrib.layers.xavier_initializer())
		biases=tf.get_variable('biases',shape=[depth],initializer=tf.constant_initializer(0.0))

		conv=tf.nn.conv3d(input,kernel,strides=[1,stride,stride,stride,1],padding='VALID')

	return tf.nn.relu(conv+biases)

def pool(input,kernel_size,scope_name):
	with tf.variable_scope(scope_name, reuse=tf.AUTO_REUSE) as scope:

		pool=tf.nn.max_pool3d(input,ksize=[1,kernel_size,kernel_size,kernel_size,1],strides=[1,kernel_size,kernel_size,kernel_size,1],padding='SAME')

	return pool

def fully_connected(input,size,scope_name):
	with tf.variable_scope(scope_name, reuse=tf.AUTO_REUSE) as scope:
		weights=tf.get_variable('weights',shape=[input.get_shape()[1],size],initializer=tf.contrib.layers.xavier_initializer())
		biases=tf.get_variable('biases',shape=[size],initializer=tf.constant_initializer(0.0))

	return tf.matmul(input,weights)+biases


def fully_connected_relu(input,size,scope_name):

	return tf.nn.relu(fully_connected(input,size,scope_name))


#construct computation graph of the CNN
def model_pass(input,training):
	#32*32*32*1
	conv1=conv_relu(input,kernel_size=5,depth=32,stride=2,scope_name='conv1') #14*14*14*32
	print(conv1.shape)
	conv2=conv_relu(conv1,kernel_size=3,depth=32,stride=1,scope_name='conv2') #12*12*12*32
	print(conv2.shape)
	pool2=pool(conv2,kernel_size=2,scope_name='pool2')    #6*6*6*32
	pool2=tf.cond(training,lambda:tf.nn.dropout(pool2,keep_prob=0.8),lambda:pool2)

	#flatten the output
	print(pool2.shape)
	shape=pool2.get_shape().as_list()
	print(shape[1]*shape[2]*shape[3]*shape[4])
	flattened=tf.reshape(pool2,[-1,shape[1]*shape[2]*shape[3]*shape[4]]) #(6*6*6*32)

	fc3=fully_connected_relu(flattened,size=128,scope_name='fc3')       #128
	fc3=tf.cond(training,lambda:tf.nn.dropout(fc3,keep_prob=0.5),lambda:fc3)
	print(fc3.shape)
	fc4=fully_connected(fc3,size=num_keypoints,scope_name='fc4')     #9
	prediction=fc4
	print(prediction.shape)
	return prediction


def calc_loss(predictions,labels):
	l=np.mean(np.square(predictions-labels))

	return l

graph=tf.Graph()
with graph.as_default():
	#placeholder for input data in batches
	tf_x_batch=tf.placeholder(tf.float32,shape=(None,voxel_size,voxel_size,voxel_size,1))
	tf_y_batch=tf.placeholder(tf.float32,shape=(None,num_keypoints))

	is_training=tf.placeholder(tf.bool)

	#epoch counter
	current_epoch=tf.Variable(0)
	learning_rate=0.001
	# Training computation.
	
	predictions = model_pass(tf_x_batch, is_training)
	#loss=calc_loss(predictions,tf_y_batch)
	loss=tf.reduce_mean(tf.square(predictions-tf_y_batch))
	#Optimizer
	optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(loss)

every_epoch_to_log=1

with tf.Session(graph=graph) as session:
	session.run(tf.global_variables_initializer())
	saver=tf.train.Saver()
	train_loss_history=np.zeros(num_epochs)
	valid_loss_history=np.zeros(num_epochs)
	print("============ TRAINING =============")
	for epoch in range(num_epochs):
		current_epoch=epoch
		batch_iterator=BatchIterator(batch_size=batch_size,shuffle=True)
		for x_batch,y_batch in batch_iterator(voxels,labels):
			session.run(optimizer,feed_dict={tf_x_batch:x_batch,tf_y_batch:y_batch,is_training:True})


		#to log the losses get predictions on entire training set
		p=[]
		total_loss=0
		if(epoch%every_epoch_to_log==0):
			batch_iterator=BatchIterator(batch_size=10)  #128
			for x_batch,y_batch in batch_iterator(voxels,labels):
				#[p_batch]=session.run([logits],feed_dict={tf_x_batch:x_batch,is_training:False})
				l=session.run(loss,feed_dict={tf_x_batch:x_batch,tf_y_batch:y_batch,is_training:False})
				total_loss+=l
				#p.extend(p_batch)
			#p=np.asarray(p)
			#print(p.shape)
			#p=p.reshape(-1)
			#train_loss=calc_loss(p,onehot_labels)
			train_loss=total_loss
			train_loss_history[epoch]=train_loss
			

			
			print("--------- EPOCH %4d/%d ---------" % (epoch, num_epochs))
			print("     Train loss: %.8f" % (train_loss))
		

	writer=tf.summary.FileWriter('./graphs/rrt_convnet',graph)
	#Save weights for future use
	save_path=saver.save(session,model_path)
	print("Model file: " + save_path)

