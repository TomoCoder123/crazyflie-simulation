#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Empty

from std_srvs.srv import Trigger
import scipy.io as io

import os
import time
import csv
import numpy
import cv2
from shutil import copyfile

def rosloop():
    bug_name = 'gradient_embedded_bug'
    noise_level = 0.2
    
    amount_of_environment = 100
    amount_of_agents = 1
    
    
    rospy.wait_for_service('/indoor_gen')
    indoor_gen_srv = rospy.ServiceProxy('/indoor_gen',Trigger)
    environment_random_pub = rospy.Publisher('/random_environment', Bool, queue_size=10)
    noise_level_pub = rospy.Publisher('/noise_level', Float64, queue_size=10)
    switch_bug_pub = rospy.Publisher('/switch_bug',String, queue_size=10)
    results = {}
    environment = dict()
    made_it = []
    
    
    msg=Float64()
    msg = 0.2
    noise_level_pub.publish(msg)
    time.sleep(2)

    
    #Make directory
    timestr = time.strftime("%Y%m%d_%H%M%S")
    filename = "experiments/test_" + timestr
    os.makedirs(filename)

    time.sleep(3)
    for it in range (0,amount_of_environment):
        
        filename_environment = filename+"/environment"+format(it)
        os.makedirs(filename_environment)
        
        print(it)
        indoor_gen_srv()
        
        time.sleep(1)
        
        msg = Bool()
        msg = True
        
        environment_random_pub.publish(msg)
        
        time.sleep(1)
        
        msg = String()
        msg = bug_name
        switch_bug_pub.publish(msg)
        time.sleep(1)


        
        #results["environment{0}".format(it)] = dict()
        '''
        trajectory_data = []
        for row in csv.reader(open("/home/knmcguire/.ros/trajectory.txt"), delimiter=','):
            trajectory_data.append(row)
        '''
        

        

            
        rospy.wait_for_message('/finished_sim_matlab', Empty)
        print("sim is finished")
        
        '''
        distance_data = []
        for row in csv.reader(open("/home/knmcguire/.ros/distances.txt"), delimiter=','):
            distance_data.append(row)
        list = distance_data[len(distance_data)-1]
        print type(distance_data)
        print distance_data[len(distance_data)-1] 
        if float(list[0]) < 1.5:
            made_it.append(int(1))
        else:
            made_it.append(int(0))
        '''        
        
        did_it_make_it = []
        for row in csv.reader(open("/home/knmcguire/.ros/did_it_make_it.txt"), delimiter=','):
            did_it_make_it.append(row)
        
        #print did_it_make_it
        list = did_it_make_it[0]
        #print list
        if float(list[0]) > 0.9:
            made_it.append(int(1))
        else:
            made_it.append(int(0))
        

            
        
        time.sleep(1)
        #Save environment
        img = cv2.imread('/home/knmcguire/.ros/environment.png',0)
        cv2.imwrite(filename_environment+"/environment.png",img)
        
        copyfile('/home/knmcguire/.ros/environment_lines.txt',filename_environment+'/environment_lines.txt')
        
        #Save Trajectory
        for it_a in range(0,amount_of_agents):
            org_file = '/home/knmcguire/.ros/trajectory'+str(it_a+1)+'.txt'
            new_file = filename_environment+'/trajectory'+str(it_a+1)+'.txt'
            copyfile(org_file,new_file)
            
            org_file = '/home/knmcguire/.ros/state_distance'+str(it_a+1)+'.txt'
            new_file = filename_environment+'/state_distance'+str(it_a+1)+'.txt'
            copyfile(org_file,new_file)
            

            
        #copyfile('/home/knmcguire/.ros/trajectory.txt',filename_environment+'/trajectory.txt')
        #copyfile('/home/knmcguire/.ros/trajectory1.txt',filename_environment+'/trajectory1.txt')
        #copyfile('/home/knmcguire/.ros/trajectory2.txt',filename_environment+'/trajectory2.txt')
        #copyfile('/home/knmcguire/.ros/trajectory3.txt',filename_environment+'/trajectory3.txt')


        #Save distance
        copyfile('/home/knmcguire/.ros/distances.txt',filename_environment+'/distances.txt')

        #Save results
        copyfile('/home/knmcguire/.ros/fitness.txt',filename_environment+'/fitness.txt')

        #results.append(environment)
        print environment
        time.sleep(1)

    numpy.savetxt(filename+'/made_it.txt',made_it,delimiter=',')

    print results
   # io.savemat('test',results)
    print("finished")

if __name__ == '__main__':

    time.sleep(2)
    rospy.init_node("experiment_script")
    #rospy.wait_for_service('get_vel_cmd')
    try:
        rosloop()
    except rospy.ROSInterruptException:
        pass
