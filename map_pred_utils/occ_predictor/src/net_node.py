#!/usr/bin/env python3
from __future__ import division
from __future__ import print_function

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from occ_predictor_msgs.srv import PredictPCL, PredictPCLResponse
import argparse
from IPython import embed

import argparse
import os, sys
import torch
import numpy as np
import time

import model_dense_nosdf

class ocp_net(object):

    def __init__(self):
        rospy.init_node('op_net', anonymous=True)
        self.init_param()
        self.weights = torch.ones(self.num_hierarchy_levels+1).float()
        
        self.model = model_dense_nosdf.GenModel(8, 1, 16, 16, 2, True, True, True, True)
        checkpoint = torch.load(self.model_path)
        self.model.load_state_dict(checkpoint['state_dict'])
        print('loaded model:', self.model_path)
        self.model.eval()
        if not self.use_cpu:
            self.model = self.model.cuda()

        self.server = rospy.Service(self.service_name, PredictPCL, self.handle_pred_pcl)
        self.sigmoid_func = torch.nn.Sigmoid()
        rospy.loginfo("net node ready!")
        rospy.spin()


    def init_param(self):
        # FIXME: use GPU is having issue
        self.use_cpu = rospy.get_param("~no_gpu", default=False)
        self.model_path = rospy.get_param("~model_path", default='../../models/no_surf.pth')

        self.num_hierarchy_levels = rospy.get_param("~num_hierarchy_levels", default=2)
        self.occ_thresord = rospy.get_param("~occ_thresord", default=0.4)
        
        self.voxel_size = rospy.get_param("~voxel_size", default=0.05)
        
        self.service_name = rospy.get_param("~service_name", default="/occ_map/pred")
        print("model path: ", self.model_path)
        print("Occ thres: ", self.occ_thresord)

    def handle_pred_pcl(self, req):
        print("inference")
        if rospy.is_shutdown():
            exit()

        time1 = time.time()
        dimx = req.dim_x
        dimy = req.dim_y
        dimz = req.dim_z
        input = req.input
        input = np.array(req.input).astype(np.float32)
        input = torch.from_numpy(input).reshape([dimx, dimy, dimz]) 
        
        if not self.use_cpu:
            input = input.cuda()



        with torch.no_grad():
            input = input.unsqueeze(0)
            _, output_occs = self.model(input, self.weights)     
            pred_occ = output_occs[-1][:,0].squeeze()
            output = pred_occ.cpu().numpy().reshape([-1]).tolist()

        return PredictPCLResponse(output)
 

if __name__ == '__main__':
    ocp_net()