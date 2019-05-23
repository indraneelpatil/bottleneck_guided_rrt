#!/usr/bin/env python
#from nolearn.lasagne import BatchIterator
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
#from sklearn.cross_validation import train_test_split
import utils.data_helper as helper
import utils.visualization as viewer

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import numpy as np
import ros_numpy
import pptk

os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

model_path = "/home/indraneel/tf-demo/VoxNet/rrt_multi_input_checkpoint/model.ckpt"

global i
i=0

global start_point
global end_point
global s
global g
global pub2
 
s=g=False




def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global i,s,g,start_point,end_point,pub,pub2
    if (i==0) and s and g:
        i=i+1
        pc = ros_numpy.numpify(data)
        points=np.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        print(points.shape)
        print(points[0])
        # Before adding points to cloud use homogenous transformation matrix from world to kinect frame
        t=np.array([[-0.7379,0.8911,0.1499,0.5155],[-2.4269,0.1731,-0.2753,3.2668],[14.1288,-0.4993,4.7629,-19.9664],[0,0,0,1]])
        #print(t)
        print(start_point)
        print(end_point)
        start_transformed=np.dot(t,np.transpose(start_point))
        #print(start_transformed)
        #print(start_transformed.shape)
        end_transformed=np.dot(t,np.transpose(end_point))
        #print(end_transformed)
        #print(end_transformed.shape)
        #start_transpose=np.transpose(start_point)
        #end_transpose=np.transpose(end_point)
        #start_transformed=t*start_transpose
        #end_transformed=(t*end_transpose)
        start_new=np.transpose(start_transformed)
        end_new=np.transpose(end_transformed)
        start_final=np.delete(start_new,3)
        end_final=np.delete(end_new,3)
        start_final = (start_final).reshape((1,3))
        print(start_final)
        print(end_final)
        #start_final[0,0]=start_final[0,0]*-1
        if start_final[0,2]>-2 or start_final[0,2]<-4:
            start_final[0,2]=-2.5

        end_final = (end_final).reshape((1,3))
        #end_final[0,0]=end_final[0,0]*-1
        if end_final[0,2]>-2 or end_final[0,2]<-4:
            end_final[0,2]=-2.5
        print("The Start Point is : ",start_final)
        print(start_final.shape)
        print("The End Point is : ",end_final)
        points=np.concatenate((points,start_final),axis=0)
        points=np.concatenate((points,end_final),axis=0)


        #v=pptk.viewer(points)
        voxels, inside_points = \
                    helper.voxelize(points, voxel_size=(24,24,24), padding_size=(32,32,32), resolution=0.1)
        print(voxels.shape)
        print(inside_points.shape)
        #v_new=pptk.viewer(inside_points)
          
        #viewer.plot3DVoxel(voxels)
        #save_dir = "/home/indraneel/tf-demo/VoxNet/labelled_data/inputs" 
        #save_name = '{}/{}.npy'.format(save_dir, str(int(end_point[0,0]*100000)))  # Change both labels
        #np.save(save_name, voxels)
		#print('saved npy. {}'.format(save_name))
        
        start_point_new=np.delete(start_point,3)
        end_point_new=np.delete(end_point,3)
        start_end_points=np.concatenate((start_point_new,end_point_new),axis=0)
        #save_dir2 = "/home/indraneel/tf-demo/VoxNet/labelled_data/start_end_points" 
        #save_name = '{}/{}.npy'.format(save_dir2, str(int(end_point[0,0]*100000)))   ### Change both labels
        #np.save(save_name, start_end_points)
        voxels=np.reshape(voxels,(-1,32,32,32,1))
        start_end_points=np.reshape(start_end_points,(1,6))
        #raw_input("Press Enter to continue...")

        with tf.Session() as sess:
        	model_saver=tf.train.import_meta_graph('/home/indraneel/tf-demo/VoxNet/rrt_multi_input_checkpoint/model.ckpt.meta')
        	ckpt=tf.train.get_checkpoint_state(os.path.dirname('/home/indraneel/tf-demo/VoxNet/rrt_multi_input_checkpoint/checkpoint'))
        	model_saver.restore(sess,ckpt.model_checkpoint_path)
        	graph = tf.get_default_graph()
        	x=graph.get_tensor_by_name('Placeholder:0')
        	endpoints=graph.get_tensor_by_name('Placeholder_1:0')
        	output=graph.get_tensor_by_name('add_3:0')
        	flag=graph.get_tensor_by_name('Placeholder_3:0')
        	print("Model restored.")
        	print('Initialized')
        	p=sess.run(output,feed_dict={x:voxels,endpoints:start_end_points,flag:False})
        	print(p)
        	print(p.shape)

        	#Publish milestone points as markers**********************
        	marker_array=MarkerArray()
        	marker = Marker()
        	marker.header.frame_id = "world"
        	marker.type = marker.SPHERE
        	marker.action = marker.ADD
        	marker.color.a = 1.0
        	marker.color.r = 1.0
        	marker.color.g = 1.0
        	marker.color.b = 0.0
        	marker.pose.orientation.w = 1.0
        	marker.scale.x = 0.09
        	marker.scale.y = 0.09
        	marker.scale.z = 0.09

        	temp=Pose()
        	temp2=Pose()
        	temp3=Pose()
        	milestone_container=PoseArray()
        	milestone_container.header.stamp = rospy.Time.now();
        	milestone_container.header.frame_id = "/world";
 
        	marker.id=1
        	marker.pose.position.x = p[0][0]
        	marker.pose.position.y = p[0][1]
        	marker.pose.position.z = p[0][2]

        	marker_array.markers.append(marker) 
        	pub.publish(marker_array)
        	
        	temp.position.x=p[0][0]
        	temp.position.y=p[0][1]
        	temp.position.z=p[0][2]
        	milestone_container.poses.append(temp)

        	marker.id=2
        	marker.pose.position.x = p[0][3]
        	marker.pose.position.y = p[0][4]
        	marker.pose.position.z = p[0][5]

        	marker_array.markers.append(marker)
        	pub.publish(marker_array)

        	temp2.position.x=p[0][3]
        	temp2.position.y= p[0][4]
        	temp2.position.z= p[0][5]
        	milestone_container.poses.append(temp2)
        	
        	marker.id=3
        	marker.pose.position.x = p[0][6]
        	marker.pose.position.y = p[0][7]
        	marker.pose.position.z = p[0][8]

        	marker_array.markers.append(marker) 
        	pub.publish(marker_array)

        	temp3.position.x=p[0][6]
        	temp3.position.y= p[0][7]
        	temp3.position.z= p[0][8]
        	milestone_container.poses.append(temp3)
        	pub2.publish(milestone_container)

        	print("Milestones are successfully published!!")

    return 0

def callback2(data):
    global start_point,end_point,s,g
    
    
    if (int(data.event_type)==5) and (data.marker_name=='1'):
        print("Start Point Received")
        start_point=np.zeros((1,4))
        start_point[0,0]=data.pose.position.x
        start_point[0,1]=data.pose.position.y
        start_point[0,2]=data.pose.position.z
        start_point[0,3]=1
        s=True
    elif (int(data.event_type)==5) and (data.marker_name=='2'):
        print("End Point Received")
        end_point=np.zeros((1,4))
        end_point[0,0]=data.pose.position.x
        end_point[0,1]=data.pose.position.y
        end_point[0,2]=data.pose.position.z
        end_point[0,3]=1
        g=True



    
def listener():
	global pub,pub2
	rospy.init_node('rrt_multiinput_prediction', anonymous=True)

	rospy.Subscriber("/compressed_points", PointCloud2, callback)
	rospy.Subscriber("/moveit_cartesian_plan_plugin/feedback",InteractiveMarkerFeedback,callback2)
	pub=rospy.Publisher('/milestone_points',MarkerArray,queue_size=10)
	pub2=rospy.Publisher('/rrt_milestone_points',PoseArray,queue_size=10)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()

