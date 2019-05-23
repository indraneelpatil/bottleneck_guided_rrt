from nolearn.lasagne import BatchIterator
import numpy as np
import tensorflow as tf
import matplotlib
matplotlib.use('TkAgg', warn = False)
#from matplotlib import pyplot
import os
import time

from pandas import DataFrame
from pandas.io.parsers import read_csv
from sklearn.utils import shuffle
from sklearn.cross_validation import train_test_split
import utils.data_helper as helper


os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

num_keypoints=9 #number of outputs of the network
batch_size=10  #32
num_epochs=100
voxel_size=32

model_path = "/home/indraneel/tf-demo/VoxNet/transfer_checkpoint/model.ckpt"

################ Load Training Data for our Application ############################3

voxels,labels,endpoints=helper.load_labelled_data()
voxels=np.reshape(voxels,(-1,32,32,32,1))
print(voxels.shape) #202,32,32,32,1
labels=np.reshape(labels,(202,-1))
print(labels.shape)    #202,9
print(endpoints.shape) #202,6
labels_endpoints=np.concatenate((labels,endpoints),axis=1) 

#We shuffle data using seed to get same result every time
voxels, labels_endpoints = shuffle(voxels, labels_endpoints, random_state=42)  # shuffle train data

# Train Test Split using sklearn
voxels_train, voxels_test, labels_endpoints_train, labels_endpoints_test = train_test_split(voxels, labels_endpoints, test_size=10, random_state=42)

print(voxels_train.shape)
print(labels_endpoints_train.shape)
print(voxels_test.shape)
print(labels_endpoints_test.shape)

input("Training and testing data Successfully loaded....Press Enter to continue...")



######################################################################################3

# CNN helper blocks
def fully_connected(input,size,scope_name):
	with tf.variable_scope(scope_name, reuse=tf.AUTO_REUSE) as scope:
		weights=tf.get_variable('weights',shape=[input.get_shape()[1],size],initializer=tf.contrib.layers.xavier_initializer())
		biases=tf.get_variable('biases',shape=[size],initializer=tf.constant_initializer(0.0))

	return tf.matmul(input,weights)+biases


def fully_connected_relu(input,size,scope_name):

	return tf.nn.relu(fully_connected(input,size,scope_name))

#construct computation graph of the CNN
def model_pass(input,endpoints,training):
	#(6*6*6*32)
	print(input.shape)

	fc3_new=fully_connected_relu(input,size=128,scope_name='fc3_new')       #128
	fc3_new=tf.cond(training,lambda:tf.nn.dropout(fc3_new,keep_prob=0.5),lambda:fc3_new)
	print(fc3_new.shape)

	#Append endpoints to fc3
	concatenatedFeatures =  tf.concat([fc3_new,endpoints],1)   
	print(concatenatedFeatures.shape)                        #134

	fc4_new=fully_connected(concatenatedFeatures,size=num_keypoints,scope_name='fc4_new')     #9
	prediction=fc4_new
	print(prediction.shape)
	return prediction


with tf.Session() as session:
	new_saver=tf.train.import_meta_graph('/home/indraneel/tf-demo/VoxNet/transfer_checkpoint/model.ckpt.meta')
	new_saver.restore(session,tf.train.latest_checkpoint('/home/indraneel/tf-demo/VoxNet/transfer_checkpoint'))
	print("Model restored.")
	print('Initialized')
	#To find names of the placeholders to restore during prediction
	graph = tf.get_default_graph()
	########################################################################################
	# Retrieve tensors on interest from old network
	tf_x_batch_new=graph.get_tensor_by_name('Placeholder:0')
	transfer_layer=graph.get_tensor_by_name('Reshape:0')
	flag=graph.get_tensor_by_name('Placeholder_2:0')

	#Declare new tensors for our modified network
	tf_endpoints_batch=tf.placeholder(tf.float32,shape=(None,6))
	tf_y_batch_new=tf.placeholder(tf.float32,shape=(None,num_keypoints))

	is_training_flag=tf.placeholder(tf.bool)

	#epoch counter
	current_epoch=tf.Variable(0)
	learning_rate=0.001
	######################################################################################
	# Pass the transfer layer through our modified network
	predictions = model_pass(transfer_layer,tf_endpoints_batch, is_training_flag)

	loss=tf.reduce_mean(tf.square(predictions-tf_y_batch_new))
	# Declare only our modified network as trainable and run the optimizer
	train_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES,scope="fc3_new|fc4_new")
	#Optimizer
	optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(loss,var_list=train_vars)

	# Print Trainable variables
	#print(tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES))
	tvars = tf.trainable_variables()
	#tvars_vals = session.run(tvars)

	#for var, val in zip(tvars, tvars_vals):
	#	print(var.name, val)  # Prints the name of the variable alongside its value.
	#train_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES,scope="fc3|fc4")
	for var in train_vars:
		print(var.name)

	every_epoch_to_log=1
	#####################################################################################
	## Retrain the new layers with manually labelled data

	session.run(tf.global_variables_initializer())
	saver=tf.train.Saver()
	train_loss_history=np.zeros(num_epochs)
	valid_loss_history=np.zeros(num_epochs)
	print("============ TRAINING =============")
	for epoch in range(num_epochs):
		current_epoch=epoch
		batch_iterator=BatchIterator(batch_size=batch_size,shuffle=True)
		for x_batch,y_end_batch in batch_iterator(voxels_train,labels_endpoints_train):
			y_batch,x2_batch=np.split(y_end_batch,[9],axis=1)
			session.run(optimizer,feed_dict={tf_x_batch_new:x_batch,tf_endpoints_batch:x2_batch,tf_y_batch_new:y_batch,is_training_flag:True,flag:False})


		#to log the losses get predictions on entire training set
		p=[]
		total_loss=0
		if(epoch%every_epoch_to_log==0):
			batch_iterator=BatchIterator(batch_size=20)  #128
			for x_batch,y_end_batch in batch_iterator(voxels_train,labels_endpoints_train):
				#[p_batch]=session.run([logits],feed_dict={tf_x_batch:x_batch,is_training:False})
				y_batch,x2_batch=np.split(y_end_batch,[9],axis=1)
				l=session.run(loss,feed_dict={tf_x_batch_new:x_batch,tf_endpoints_batch:x2_batch,tf_y_batch_new:y_batch,is_training_flag:False,flag:False})
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
		

	# Log losses on the test set
	total_test_loss=0
	
	batch_iterator=BatchIterator(batch_size=5)  #128
	for x_batch,y_end_batch in batch_iterator(voxels_test,labels_endpoints_test):
		#[p_batch]=session.run([logits],feed_dict={tf_x_batch:x_batch,is_training:False})
		y_batch,x2_batch=np.split(y_end_batch,[9],axis=1)
		l=session.run(loss,feed_dict={tf_x_batch_new:x_batch,tf_endpoints_batch:x2_batch,tf_y_batch_new:y_batch,is_training_flag:False,flag:False})
		total_test_loss+=l
		#p.extend(p_batch)
	#p=np.asarray(p)
	#print(p.shape)
	#p=p.reshape(-1)
	#train_loss=calc_loss(p,onehot_labels)
	test_loss=total_test_loss
			

	print("     Test loss: %.8f" % (test_loss))

	writer=tf.summary.FileWriter('./graphs/transfer_convnet',graph)
	#Save weights for future use
	save_path=saver.save(session,model_path)
	print("Model file: " + save_path)


