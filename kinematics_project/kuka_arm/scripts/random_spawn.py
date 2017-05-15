#! /usr/bin/env python
#TODO: needs more comments

#import ros packages
import yaml
import rospy
import rospkg
from random import randint

rospack = rospkg.RosPack()
config_path=rospack.get_path('kuka_arm')+"/config/target_spawn_locations.yaml"

select=randint(1,9)
print"Target will spawn at location# %d" %select
with open(config_path, 'r') as doc:
    location=yaml.load(doc)

x_select=location["locations"][select-1][0]
y_select=location["locations"][select-1][1]
z_select=location["locations"][select-1][2]

#set bin location param
rospy.set_param('target_drop_location',{'x':0.0,'y':2.5,'z':0.0})

#set selected location param
rospy.set_param('target_spawn_location',{'x':x_select,'y':y_select,'z':z_select})

#hacky set all args as param
hacky_arg = "-urdf -param target_description -x "+str(x_select)+" -y "+str(y_select)+" -z "+str(z_select)+" -model target_model"
rospy.set_param('hacky_target_spawn_location',hacky_arg)
