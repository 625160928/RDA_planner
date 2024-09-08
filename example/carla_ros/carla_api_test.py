#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

import matplotlib.pyplot as plt

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
import random
import time
import rospy
from visualization_msgs.msg import MarkerArray
from rosmap import euler_from_quaternion
from rosmap import create_obj_map_yaml

def carla_get_car_location(world,car_type_id='vehicle.tesla.model3'):
    retry_count = 20
    while retry_count > 0:
        actors_list = world.get_actors()  # .find('vehicle')
        if len(actors_list) > 0:
            break
        else:
            time.sleep(0.2)
            retry_count -= 1
    id = -1
    # print(len(actors_list))
    for actor in actors_list:
        if car_type_id in actor.type_id:
            # print(actor.attributes['role_name'])
            # print(actor,actor.type_id)
            id = actor.id
    # print(actors_list,type(actors_list))
    if id == -1:
        print('can not find vehicle.tesla.model3')
        exit()
    actor = world.get_actor(id)

    location = actor.get_location()
    # print('location ', location.x, location.y, location.z)
    transform=actor.get_transform()
    l=transform.location
    r=transform.rotation
    # print('tf_location xyz ',l.x,l.y,l.z)
    # print('tf_rotation pitch yaw roll ',r.pitch,r.yaw,r.roll)
    return [location.x, location.y, location.z,r.yaw/180*math.pi]


def carla_get_car_id(world,car_type_id='vehicle.tesla.model3'):
    retry_count = 20
    while retry_count > 0:
        actors_list = world.get_actors()  # .find('vehicle')
        if len(actors_list) > 0:
            break
        else:
            time.sleep(0.2)
            retry_count -= 1
    id = -1
    # print(len(actors_list))
    for actor in actors_list:
        if car_type_id in actor.type_id:
            # print(actor.attributes['role_name'])
            # print(actor,actor.type_id)
            id = actor.id
    # print(actors_list,type(actors_list))
    if id == -1:
        print('can not find '+car_type_id)
        exit()
        return -1
    return id




def carla_location_to_map_location(car_location,map_name):
    car_location_3d = [-car_location[1], car_location[0], car_location[3]]
    if 'town04' in map_name:
        # car_location_3d[0] -= 100
        # car_location_3d[1] += 60
        car_location_3d[2] += math.pi/2
    return car_location_3d

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(200.0)
    world = client.get_world()

    map_name='town04_r1y'
    # map_name='town04_r1'
    json_file='./map/'+map_name+'.json'

    ego_car_id=carla_get_car_id(world)




    while True:
        st_time=time.time()


        # car_location=carla_get_car_location(world) #x,y,z,rad
        # print(car_location)

        #carla ros xy need change
        # car_location_3d=carla_location_to_map_location(car_location,map_name)

        car_location=ros_get_car_position(ego_car_id)#x,y,z,rad
        # print('car_location',car_location)
        car_location_3d =[car_location[0],car_location[1],car_location[3]]
        # print('car_location_3d',car_location_3d)
        car_goal=[car_location_3d[0],car_location_3d[1]-100,car_location[3]]
        # print(car_location_3d)
        # plt.clf()

        yaml_path=create_obj_map_yaml(json_file,car_location_3d,car_goal,view_range=[150,150])
        ed_time=time.time()
        print('get map time is ',ed_time-st_time)

        # plt.pause(0.5)


def ros_get_car_position(car_id):

    datas=rospy.wait_for_message('/carla/markers', MarkerArray, timeout=None)
    for data in datas.markers:
        # print(data)
        frame_id=data.header.frame_id
        ego_id=data.id
        pose=data.pose
        scale=data.scale
        roll_x, pitch_y, roll_z=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        # print(roll_x*180/math.pi, pitch_y*180/math.pi, roll_z*180/math.pi)
        rt_pose=[pose.position.x,pose.position.y,pose.position.z,roll_z]
        if ego_id==car_id:
            return rt_pose
    return None





if __name__ == '__main__':

    rospy.init_node('py_carla_pose_subscribe', anonymous=True)
    main()

