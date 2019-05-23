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


num_classes=14  #number of outputs
batch_size=32
num_epochs=20
voxel_size=32

DATA='/home/indraneel/tf-demo/VoxNet/npy_generated'
model_path = "/home/indraneel/tf-demo/VoxNet/transfer_checkpoint/model.ckpt"

def indices_to_one_hot(data, nb_classes):
    """Convert an iterable of indices to one-hot encoded labels."""
    targets = np.array(data).reshape(-1)
    return np.eye(nb_classes)[targets]

# Training data collector
voxels, labels = helper.load_data_from_npy(DATA, mode='training')
#voxels=tf.reshape(voxels,[-1,32,32,32,1])
voxels=np.reshape(voxels,(-1,32,32,32,1))
#onehot_labels = tf.one_hot(indices=tf.cast(labels, tf.int32), depth=num_classes)
onehot_labels=indices_to_one_hot(labels,14)
print(voxels.shape) #(5521, 32, 32, 32, 1)
print(onehot_labels.shape) #(5521, 14)
print(onehot_labels[0])


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
	#print(conv1.shape)
	conv2=conv_relu(conv1,kernel_size=3,depth=32,stride=1,scope_name='conv2') #12*12*12*32
	#print(conv2.shape)
	pool2=pool(conv2,kernel_size=2,scope_name='pool2')    #6*6*6*32
	pool2=tf.cond(training,lambda:tf.nn.dropout(pool2,keep_prob=0.8),lambda:pool2)

	#flatten the output
	#print(pool2.shape)
	shape=pool2.get_shape().as_list()
	#print(shape[1]*shape[2]*shape[3]*shape[4])
	flattened=tf.reshape(pool2,[-1,shape[1]*shape[2]*shape[3]*shape[4]]) #(6*6*6*32)

	fc3=fully_connected_relu(flattened,size=128,scope_name='fc3')       #128
	fc3=tf.cond(training,lambda:tf.nn.dropout(fc3,keep_prob=0.5),lambda:fc3)

	fc4=fully_connected_relu(fc3,size=num_classes,scope_name='fc4')     #14
	logits=fc4
	print(logits.shape)
	return logits

def predicted_class(logits):

	return tf.argmax(input=logits, axis=1)

#Define loss function
def calc_loss(logits,labels):
	#Convert to one hot encodings
	#onehot_labels = tf.one_hot(indices=tf.cast(labels, tf.int32), depth=num_classes)
	loss=tf.math.reduce_mean(tf.losses.softmax_cross_entropy(labels,logits))
	tf.summary.scalar("loss", loss)
	return loss


graph=tf.Graph()
with graph.as_default():
	#placeholder for input data in batches
	tf_x_batch=tf.placeholder(tf.float32,shape=(None,voxel_size,voxel_size,voxel_size,1))
	tf_y_batch=tf.placeholder(tf.float32,shape=(None,num_classes))

	is_training=tf.placeholder(tf.bool)

	#epoch counter
	current_epoch=tf.Variable(0)
	learning_rate=0.001
	# Training computation.
	
	logits = model_pass(tf_x_batch, is_training)
	loss=calc_loss(logits,tf_y_batch)
	#loss=tf.reduce_mean(tf.square(predictions-tf_y_batch))
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
		for x_batch,y_batch in batch_iterator(voxels,onehot_labels):
			session.run(optimizer,feed_dict={tf_x_batch:x_batch,tf_y_batch:y_batch,is_training:True})


		#to log the losses get predictions on entire training set
		p=[]
		total_loss=0
		if(epoch%every_epoch_to_log==0):
			batch_iterator=BatchIterator(batch_size=128)
			for x_batch,y_batch in batch_iterator(voxels,onehot_labels):
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
		

	writer=tf.summary.FileWriter('./graphs/voxels_convnet2',graph)
	#Save weights for future use
	save_path=saver.save(session,model_path)
	print("Model file: " + save_path)