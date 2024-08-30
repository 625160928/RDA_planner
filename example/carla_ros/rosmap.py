import time
import json
import rospy
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
import math
import numpy as np

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


def markers_static_callback(data):
    print(data)

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
        num=int(max(sy,sx)*2/reso)*2

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

def main():
    st_time = time.time()
    rospy.init_node('py_RDA_map_creater', anonymous=True)
    # rospy.Subscriber('/carla/markers/static', MarkerArray, markers_static_callback)

    msg=rospy.wait_for_message('/carla/markers/static', MarkerArray, timeout=None)

    map_name='town01'

    save_dir='./map/'
    # print(data)
    road_list = []
    for data in msg.markers:
        if data.ns == 'Roads':
            # print(data.ns)
            road_list.append(data)

    # show_road(road_list)
    grid_map,config=gen_grid_map(road_list,reso=0.1)
    # print(grid_map)

    # 将Python对象转换为JSON字符串
    json_data = json.dumps(config)

    # 将JSON字符串保存到文件
    with open( save_dir+map_name+'.json', 'w') as file:
        file.write(json_data)


    plt.imshow(grid_map, cmap='Greys', origin='lower')
    plt.savefig(save_dir+map_name+'.png')
    # plt.colorbar()
    plt.show()


    ed_time=time.time()
    print('use time ',ed_time - st_time)

if __name__ == '__main__':

    main()

