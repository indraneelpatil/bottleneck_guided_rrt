#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import InteractiveMarkerFeedback
import numpy as np
import ros_numpy
import pptk
import utils.data_helper as helper
import utils.visualization as viewer

global i
i=0

global start_point
global end_point
global s
global g
 
t=np.array([[-0.7379,0.8911,0.1499,0.5155],[-2.4269,0.1731,-0.2753,3.2668],[14.1288,-0.4993,4.7629,-19.9664],[0,0,0,1]])
print(t)
#start_point=np.matrix([[0,0,0,1]])
#end_point=np.matrix([[0,0,0,1]])

s=g=False

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global i,s,g,start_point,end_point
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


        v=pptk.viewer(points)
        voxels, inside_points = \
                    helper.voxelize(points, voxel_size=(24,24,24), padding_size=(32,32,32), resolution=0.1)
        print(voxels.shape)
        print(inside_points.shape)
        #v_new=pptk.viewer(inside_points)
          
        #viewer.plot3DVoxel(voxels)
        save_dir = "/home/indraneel/tf-demo/VoxNet/labelled_data/inputs" 
        save_name = '{}/{}.npy'.format(save_dir, str(int(end_point[0,0]*100000)))  # Change both labels
        np.save(save_name, voxels)

        print('saved npy. {}'.format(save_name))
        
        start_point_new=np.delete(start_point,3)
        end_point_new=np.delete(end_point,3)
        start_end_points=np.concatenate((start_point_new,end_point_new),axis=0)
        save_dir2 = "/home/indraneel/tf-demo/VoxNet/labelled_data/start_end_points" 
        save_name = '{}/{}.npy'.format(save_dir2, str(int(end_point[0,0]*100000)))   ### Change both labels
        np.save(save_name, start_end_points)

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

    rospy.init_node('voxelize_cloud', anonymous=True)

    rospy.Subscriber("/compressed_points", PointCloud2, callback)
    rospy.Subscriber("/moveit_cartesian_plan_plugin/feedback",InteractiveMarkerFeedback,callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()