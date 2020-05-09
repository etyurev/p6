
import numpy as np
import transformations as tf
from scipy.spatial.transform import Rotation as R

theta_x=1.56                   # rot_x=1.56 rad

theta_y_r=0.51                    # rot_y=0.95 rad
theta_z_r=1.56                   # rot_z=-1.55 rad  
theta_y_l=0.51                    
theta_z_l=2*1.57     
x_tr_r=np.array((0.6,-3.5,3.2,1))  # ce_jacket position in gazebo x=0.6; y=1.2; z=4.2; 
T_x_r=tf.translation_matrix(x_tr_r)
x_tr_l=np.array((3.6,-0.3,3.2,1))  # ce_jacket position in gazebo x=0.6; y=1.2; z=4.2; 
T_x_l=tf.translation_matrix(x_tr_l)

# I=tf.identity_matrix()
Rot_x=tf.rotation_matrix(theta_x,(1,0,0))
Rot_y_r=tf.rotation_matrix(theta_y_r,(0,1,0))
Rot_z_r=tf.rotation_matrix(theta_z_r,(0,0,1))
Rot_y_l=tf.rotation_matrix(theta_y_l,(0,1,0))
Rot_z_l=tf.rotation_matrix(theta_z_l,(0,0,1))



#Transformation matrixes from origin to camera
print (np.matrix.round(((T_x_r).dot(Rot_z_r).dot(Rot_y_r)),3))
print (np.matrix.round(((T_x_l).dot(Rot_z_l).dot(Rot_y_l)),3))

#Transformation matrixes from origin to camera with rotation about x for t-joint (not sure is it correct)
#print (np.matrix.round(((Rot_x).dot(T_x_r).dot(Rot_z_r).dot(Rot_y_r)),3))
#print (np.matrix.round(((Rot_x).dot(T_x_l).dot(Rot_z_l).dot(Rot_y_l)),3))

#camera placed from the right side from ce_jacket
camera_pose_right=[0.6,1.2,4.2,0,0.95,-1.56]
#camera placed from the right side from ce_jacket
camera_pose_left=[0.6,-2.2,4.2,0,0.95,1.56]
#ce_jacket pose
ce_jacket_pose=[0,0,0,1.56,0,0]




'''
r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
print(r.as_matrix())
'''