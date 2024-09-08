import copy
import time
import json
import rospy
import matplotlib.pyplot as plt
import math
import numpy as np
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from map_process import *


tf_dict=dict()

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    roll_z = math.atan2(t3, t4)

    return roll_x, pitch_y, roll_z

markers_pose=dict()

def markers_callback(datas):
    print('---------------------------------')
    # print(datas)
    for data in datas.markers:
        # print(data)
        frame_id=data.header.frame_id
        ego_id=data.id
        pose=data.pose
        scale=data.scale
        print(type(ego_id),frame_id,ego_id,pose,scale)
        markers_pose[ego_id]=[frame_id,pose,scale]



def ex_msg_to_rec(x,y,l,w,rad):
    # print('----------------------')
    # print(x,y,l,w,rad)
    pose=np.array([[x,y],[x,y],[x,y],[x,y]])
    tr=np.array([[-l/2,w/2],[l/2,w/2],[l/2,-w/2],[-l/2,-w/2]])
    tp=np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
    # print('tr',tr,'\ntp',tp,'\ndot',np.dot(tr,tp.T))
    npose=pose+np.dot(tr,tp.T)

    # print(npose)
    # pass
    return npose


def check_if_point_in_rec(point,rec_list):
    return isInterArea(point,AreaPoint=rec_list)
    # return False


#
def isInterArea(testPoint,AreaPoint):#testPoint为待测点[x,y]
    LBPoint = AreaPoint[0]#AreaPoint为按顺时针顺序的4个点[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
    LTPoint = AreaPoint[1]
    RTPoint = AreaPoint[2]
    RBPoint = AreaPoint[3]
    a = (LTPoint[0]-LBPoint[0])*(testPoint[1]-LBPoint[1])-(LTPoint[1]-LBPoint[1])*(testPoint[0]-LBPoint[0])
    b = (RTPoint[0]-LTPoint[0])*(testPoint[1]-LTPoint[1])-(RTPoint[1]-LTPoint[1])*(testPoint[0]-LTPoint[0])
    c = (RBPoint[0]-RTPoint[0])*(testPoint[1]-RTPoint[1])-(RBPoint[1]-RTPoint[1])*(testPoint[0]-RTPoint[0])
    d = (LBPoint[0]-RBPoint[0])*(testPoint[1]-RBPoint[1])-(LBPoint[1]-RBPoint[1])*(testPoint[0]-RBPoint[0])
    #print(a,b,c,d)
    if (a>0 and b>0 and c>0 and d>0) or (a<0 and b<0 and c<0 and d<0):
        return True
    else:
        return False


def gen_grid_map(road_list,reso=0.1):

    rec_list=[]
    min_x,max_x=road_list[0].pose.position.x,road_list[0].pose.position.x
    min_y,max_y=road_list[0].pose.position.y,road_list[0].pose.position.y
    for d in road_list:
        r,y,rad=euler_from_quaternion([d.pose.orientation.x,d.pose.orientation.y,
                                     d.pose.orientation.z,d.pose.orientation.w])
        px=d.pose.position.x
        py=d.pose.position.y
        sx=d.scale.x
        sy=d.scale.y
        if sx<0 or sy<0:
            print('scale is lower than zero !!!!!!  \n',d)
        rec_list.append([px,py,sx,sy,rad])
        if px-sx/2<min_x:
            min_x=px-sx/2
        if px+sx/2>max_x:
            max_x=px+sx/2
        if py-sy/2<min_y:
            min_y=py-sy/2
        if py+sy/2>max_y:
            max_y=py+sy/2
    min_x-=1
    max_x+=1
    min_y-=1
    max_y+=1
    lx=int((max_x-min_x)/reso)
    ly=int((max_y-min_y)/reso)
    # print(min_x,max_x,min_y,max_y,'-----',lx,ly)
    stx=min_x
    sty=min_y
    grid_map=np.ones((lx,ly))
    total_len=len(rec_list)
    count=0
    # print(total_len)
    for px,py,sx,sy,rad in rec_list:
        # break
        count+=1
        # if count<=714:
        #     continue
        # print('------------------------------------')
        num=int(max(sy,sx)*2/reso)*2+1

        print(count,round(100*count/total_len,2),'%')
        if num*num<lx*ly:
            # print( count,px,py,sx,sy,rad,'===',num)
            # print(np.linspace(-sx/2,sx/2,num),len(np.linspace(-sx/2,sx/2,num)))
            # print(np.linspace(-sy/2,sy/2,num),len(np.linspace(-sy/2,sy/2,num)))

            if count==716:
                continue

            offset_list=[]
            for x in np.linspace(-sx/2,sx/2,num):
                for y in np.linspace(-sy/2,sy/2,num):
                    offset_list.append([x,y])
            np_offset_list=np.array(offset_list)
            tp=np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
            # print(np_offset_list.shape)
            # print()
            npose = np.array([[px,py]]*(num*num)) + np.dot(np_offset_list, tp.T)
            # print(npose)
            for x,y in npose:
                if x-stx>=0 and y-sty>=0:

                    grid_map[int((x-stx)/reso)][int((y-sty)/reso)]=0
                # print(int((x-stx)/reso),int((y-sty)/reso))
        else:
            # print('????')
            nrec=ex_msg_to_rec(px,py,sx,sy,rad)
            point_count=0
            for x in range(int(min(nrec[:,0])),int(max(nrec[:,0]))):
                for y in range(int(min(nrec[:,1])),int(max(nrec[:,1]))):
                    point_count+=1
                    # if check_if_point_in_rec([stx+x*reso,sty+y*reso],nrec)==True:
                    #     grid_map[x][y]=0
                    if check_if_point_in_rec([x,y],nrec)==True:
                        grid_map[int((x - stx) / reso)][int((y - sty) / reso)] = 0

                    # print(point_count,x,y)
            # print(nrec,[x,y])

        # break
        # if round(100*count/total_len,2)>99.5:
        #     print(count,total_len)
        #     break
    print('gen grid map finish')
    config_dict=dict()
    config_dict['zero_position']=[stx,sty]
    config_dict['reso']=reso
    config_dict['shape']=[lx,ly]
    grid_map=grid_map.T
    config_dict['map']=grid_map.tolist()


    return grid_map,config_dict


def show_road(road_list):

    # print(MarkerArray(),type(data.markers))
    # print(road_list[0])
    for d in road_list:
        # d=road_list[0]
        # if d.pose.position.x>43.5 and d.pose.position.x<44 and d.pose.position.y>-130 and d.pose.position.y<-128.5:
        #     print(d)
        r, y, p = euler_from_quaternion([d.pose.orientation.x, d.pose.orientation.y,
                                         d.pose.orientation.z, d.pose.orientation.w])
        # print(len(road_list),[d.pose.orientation.x,d.pose.orientation.y,
        #                              d.pose.orientation.z,d.pose.orientation.w],'----',r,y,p)
        rec = ex_msg_to_rec(d.pose.position.x, d.pose.position.y, d.scale.x, d.scale.y, p).tolist()
        rec = np.array(rec + [rec[0]])
        # rx=rec[:,0]
        # print(rec)
        # print(rec[:,0],rec[:,1])
        # print(rx)

        plt.plot(rec[:, 0], rec[:, 1])


    plt.show()


def save_ros_map(reso=0.1):

    msg=rospy.wait_for_message('/carla/markers/static', MarkerArray, timeout=None)

    map_name='town04_r1y'

    save_dir='./map/'
    # print(data)
    road_list = []
    for data in msg.markers:
        if data.ns == 'Roads':
            # print(data.ns)
            road_list.append(data)

    # show_road(road_list)
    grid_map,config=gen_grid_map(road_list,reso=reso)
    # print(grid_map)
    # grid_map

    # 将Python对象转换为JSON字符串
    json_data = json.dumps(config)

    # 将JSON字符串保存到文件
    with open( save_dir+map_name+'.json', 'w') as file:
        file.write(json_data)


    plt.imshow(grid_map, cmap='Greys', origin='lower')
    plt.savefig(save_dir+map_name+'.png')
    # plt.colorbar()
    plt.show()


def get_center_map(grid_map_msg, car_position, x_range, y_range):
    reso=grid_map_msg['reso']
    lx,ly=grid_map_msg['shape']
    grid_map=np.array(grid_map_msg['map'])
    stx,sty=grid_map_msg['zero_position']
    car_pose_x_index=int((car_position[0]-stx)/reso)
    car_pose_y_index=int((car_position[1]-sty)/reso)
    x_half_index=int(x_range/reso/2)
    y_half_index=int(y_range/reso/2)
    x_index_range=[max(0,car_pose_x_index-x_half_index),min(lx,car_pose_x_index+x_half_index)]
    y_index_range=[max(0,car_pose_y_index-y_half_index),min(ly,car_pose_y_index+y_half_index)]

    # print(grid_map.shape,lx,ly)
    # # print(lx,ly)
    # print(stx,sty,reso)
    # print(car_position,car_pose_x_index,car_pose_y_index)
    # print(x_index_range,y_index_range)
    # new_map=grid_map[x_index_range[0]:x_index_range[1],y_index_range[0]:y_index_range[1]]
    new_map=grid_map[y_index_range[0]:y_index_range[1],x_index_range[0]:x_index_range[1]]
    # print(len(new_map),len(new_map[0]))

    return np.array(new_map),reso,max(car_position[0]-x_range/2,stx),max(car_position[1]-y_range/2,sty)


def save_yaml_map(center_map,center_map_size,start_position,goal_position,view_range,yaml_save_path=''):

    total_dict=dict()

    world_dict=dict()

    # world_dict['height'] =view_range[0]
    # world_dict['width'] =view_range[1]

    world_dict['height'] =view_range[1]
    world_dict['width'] =view_range[0]

    world_dict['step_time'] =0.1
    world_dict['sample_time'] =0.1
    world_dict['offset'] =[start_position[0]-view_range[0]/2,start_position[1]-view_range[1]/2]
    world_dict['collision_mode'] ='unobstructed'  # 'stop', 'unobstructed', 'reactive'
    world_dict['control_mode'] ="auto"
    total_dict['world']=world_dict


    robot_dict=dict()
    robot_dict['kinematics'] ={'name': 'acker'}
    robot_dict['shape'] ={'name': 'rectangle', 'length': 4.6, 'width': 1.6,
                          'wheelbase': 3}
    robot_dict['state'] = [start_position[0],start_position[1],start_position[2], 0]
    robot_dict['goal'] =goal_position
    robot_dict['vel_min'] =[-8, -1]
    robot_dict['vel_max'] =[8, 1]
    robot_dict['goal_threshold'] =0.3
    plot_dict=dict()
    plot_dict['show_trail']=True
    plot_dict['show_goal']=True
    robot_dict['plot'] =plot_dict
    total_dict['robot']=robot_dict


    obs_dict=dict()
    obs_dict['number']=len(center_map)
    obs_dict['distribution'] ={'name': 'manual'}
    obs_dict['shape'] =[]
    obs_dict['state'] =[]
    for x,y,sx,sy in center_map:
        obs_dict['shape'].append({'name': 'rectangle','length':sx,'width':sy})
        obs_dict['state'].append([x,y,0])
    total_dict['obstacle']=[obs_dict]

    if yaml_save_path=='':
        yaml_save_path='./map/writeYamlData.yaml'

    with open(yaml_save_path, 'w', encoding='utf-8') as f:
        yaml.dump(data=total_dict, stream=f, allow_unicode=True)

    return yaml_save_path


def grid_map_to_object_map(grid_map,reso,stx,sty):
    obj_list=[]
    center_list=[]
    n=len(grid_map)
    m=len(grid_map[0])
    # print(reso,stx,sty,n,m)
    cmap=copy.deepcopy(grid_map)
    for i in range(n):
        for j in range(m):
            if cmap[i][j]==0:
                continue
            # print('-------------------')
            # print(i,j)
            top,button=i,i
            left,right=j,j
            for k in range(j+1,m):
                if cmap[i][k]!=0:
                    right=k
                else:
                    break
            # print( left,right,top,button)
            # print(cmap[top,left:right+1])
            for ii in range(top+1,n):
                pd=True
                for jj in range(left,right+1):
                    if cmap[ii][jj]==0:
                        pd=False
                        break
                if pd==True:
                    button=ii
                else:
                    break
            for ii in range(left,right+1):
                for jj in range(top,button+1):
                    cmap[jj][ii]=0
            obj_list.append([left*reso+stx,right*reso+stx,top*reso+sty,button*reso+sty])
            # obj_list.append([left*reso+stx,right*reso+stx,top*reso+sty,button*reso+sty])
            # center_list.append([left+right/2,top,button])
            cen_point=[stx+(left+right)/2*reso,sty+(top+button)/2*reso]
            scale_len=[(right-left)*reso,(button-top)*reso]
            center_list.append(cen_point+scale_len)




    return obj_list,center_list


def read_map(map_path):
    data=read_json_file(map_path)
    # print(data)
    reso=data['reso']
    lx,ly=data['shape']
    grid_map=data['map']
    stx,sty=data['zero_position']





    # car_position=[100,11,math.pi/2]
    # view_range=[100,200]
    # car_goal_position=[23,75,math.pi]
    car_position=[-100,-300,math.pi/2]
    view_range=[100,200]
    car_goal_position=[-100,-220,math.pi/2]



    # plt.subplot(1,2,1)
    # plt.imshow(grid_map, cmap='Greys', origin='lower')
    # plt.scatter(car_position[0]-stx,car_position[1]-sty)
    # px,py=car_position[0]-stx,car_position[1]-sty
    # plt.plot([px-view_range[0]/2,px-view_range[0]/2,px+view_range[0]/2,px+view_range[0]/2,px-view_range[0]/2],
    #          [py-view_range[1]/2,py+view_range[1]/2,py+view_range[1]/2,py-view_range[1]/2,py-view_range[1]/2],color='blue')


    # plt.show()

    car_center_map,center_map_reso,center_map_stx,center_map_sty=get_center_map(
        data,car_position,view_range[0],view_range[1])
    # print(car_center_map.shape)
    # plt.subplot(1,2,2)
    # plt.imshow(car_center_map, cmap='Greys', origin='lower')
    # plt.show()
    print('center_map_reso,center_map_stx,center_map_sty',center_map_reso,center_map_stx,center_map_sty)
    object_map,center_map=grid_map_to_object_map(car_center_map,center_map_reso,center_map_stx,center_map_sty)

    save_yaml_map(center_map,[center_map_reso*len(car_center_map),center_map_reso*len(car_center_map[0])],car_position,car_goal_position,view_range)

    # print('object_map',object_map)
    # plt.clf()
    # plt.subplot(1,2,2)
    # plt.imshow(grid_map, cmap='Greys', origin='lower')
    # for l,r,t,b in object_map:
    #     plt.plot([l-stx,r-stx,r-stx,l-stx,l-stx],
    #              [b-sty,b-sty,t-sty,t-sty,b-sty])
    # plt.show()


def create_obj_map_yaml(map_json_file,car_position,car_goal,view_range,yaml_save_path=''):
    data=read_json_file(map_json_file)

    #region draw base all map

    # reso=data['reso']
    # lx,ly=data['shape']
    # grid_map=data['map']
    # stx,sty=data['zero_position']
    # # print(stx,sty)
    # plt.clf()
    # plt.subplot(1,2,1)
    # plt.imshow(grid_map, cmap='Greys', origin='lower')
    # px,py=(car_position[0]-stx)/reso,(car_position[1]-sty)/reso
    #
    # # px,py=py,px
    #
    # plt.scatter(px,py,color='blue')
    # plt.scatter(py,px,color='blue')

    # px,py=(-stx)/reso,(-sty)/reso
    # plt.scatter(px,py,color='green')


    # tmp_pose=[0,0]
    # plt.scatter((tmp_pose[0]-stx)/reso,(tmp_pose[1]-sty)/reso,color='red')

    # #y line
    # for i in range(ly):
    #     if i>ly/2:
    #         plt.scatter(i,-sty,color='black')
    #     else:
    #         plt.scatter(i,-sty)
    # #x line
    # for i in range(lx):
    #     if i>lx/2:
    #         plt.scatter(-stx,i,color='red')
    #     else:
    #         plt.scatter(-stx,i)


    # tmp_pose=[428,865]
    # plt.scatter(tmp_pose[0],tmp_pose[1],color='red')
    # print('tmp_change',tmp_pose[0]*reso+stx,tmp_pose[1]*reso+sty)
    #
    #
    # plt.plot([px-view_range[0]/2,px-view_range[0]/2,px+view_range[0]/2,px+view_range[0]/2,px-view_range[0]/2],
    #          [py-view_range[1]/2,py+view_range[1]/2,py+view_range[1]/2,py-view_range[1]/2,py-view_range[1]/2],color='blue')


    # plt.show()
    #endregion

    #get object map
    car_center_map,center_map_reso,center_map_stx,center_map_sty=get_center_map(
        data,car_position,view_range[0],view_range[1])

    object_map,center_map=grid_map_to_object_map(car_center_map,center_map_reso,center_map_stx,center_map_sty)


    #region draw object map
    # plt.subplot(1,2,2)
    # plt.imshow(grid_map, cmap='Greys', origin='lower')
    # for l,r,t,b in object_map:
    #     plt.plot([l-stx,r-stx,r-stx,l-stx,l-stx],
    #              [b-sty,b-sty,t-sty,t-sty,b-sty])
    # plt.show()
    #endregion



    save_path=save_yaml_map(center_map,[center_map_reso*len(car_center_map),center_map_reso*len(car_center_map[0])],
                            car_position,car_goal,view_range,yaml_save_path=yaml_save_path)


    return save_path

def tf_callback(tf_msg):
    for m in tf_msg.transforms:
        frames=[m.header.frame_id,m.child_frame_id]


        # print(frames,m)

# 读取JSON文件
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
        return data





def main():
    st_time = time.time()
    map_name='town04_r1'
    # read_map('./map/'+map_name+'.json')

    save_ros_map(reso=1)

    ed_time=time.time()
    print('use time ',ed_time - st_time)



if __name__ == '__main__':
    rospy.init_node('py_RDA_map_creater', anonymous=True)

    # rospy.Subscriber('/carla/markers', MarkerArray,markers_callback)
    # rospy.spin()
    main()



