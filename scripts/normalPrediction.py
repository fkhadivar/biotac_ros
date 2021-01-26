#!/usr/bin/env python

from biotac_ros.srv import nnmodel,nnmodelResponse
from std_msgs.msg import Float64
import rospy
import numpy as np
import torch
import torch.nn.functional as F
import os.path


def makePrediction(predData):

    ffData = predData
    
    pred_normal = model(torch.FloatTensor(ffData)).squeeze().tolist() # whatever the model prediction is

    return pred_normal





def handle_normal_prediction(req):
    data_req = req.input;
    normalxz = makePrediction(data_req)
    return nnmodelResponse(normalxz)


if __name__ == "__main__":

    rospy.init_node('nn_model_prediction')

    modelDirectory = "/home/farshad/git/bioTac/py/nn_test/models"
    modelName = os.path.join(modelDirectory,'fullModel.pt')
    model = torch.load(modelName, map_location='cpu')

    s = rospy.Service('normal_prediction', nnmodel, handle_normal_prediction)
    rospy.spin()