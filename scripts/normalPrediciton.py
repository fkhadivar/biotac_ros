#!/usr/bin/env python

from biotac_ros.srv import nnmodel,nnmodelResponse
import rospy

def handle_normal_prediction(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return nnmodelResponse(req.a + req.b)


if __name__ == "__main__":
    rospy.init_node('nn_model_prediction')
    s = rospy.Service('normal_prediction', nnmodel, handle_normal_prediction)
    print("Ready to add two ints.")
    rospy.spin()