
//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of biotac_ros.
//|
//|    biotac_ros is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    biotac_ros is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#include <mutex>
#include <fstream>
#include <pthread.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "sensor_process.h"
#include <biotac_ros/fingerMsg.h>


class BiotacRosMaster 
{
  public:
    BiotacRosMaster(ros::NodeHandle &n,double frequency):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency){
        _stop =false;
    }

    ~BiotacRosMaster(){}

    bool init(){
        
        _bioTac = std::make_shared<biotac::BiotacSensor>();

        _subBioTac = _n.subscribe("/biotac_sp_ros",1,
        &BiotacRosMaster::updateBioTac,this,ros::TransportHints().reliable().tcpNoDelay());

        // plotting
        _plotVar.data.resize(19);
        //TODO make plotting more clean and like a function that receives input from contrl unit
        _plotPublisher = _n.advertise<std_msgs::Float64MultiArray>("/hand/plotvar",1);

        _fingertipPublisher = _n.advertise<biotac_ros::fingerMsg>("/biotac/fingertip1",1);
        //todo condition here
        return true;
    }
    // run node
    void run(){
        while(!_stop && ros::ok()){ 
            _mutex.lock();
            if(_bioTac->isOk() ){

                Eigen::VectorXd plotVariable = _bioTac->getElec(1);
                for (size_t i = 0; i < plotVariable.size(); i++)
                    _plotVar.data[i] = plotVariable[i];
                _plotPublisher.publish(_plotVar);

                _fingerMsg.header.stamp = ros::Time::now();
                for (size_t i = 0; i < 3; i++)
                {
                    _fingerMsg.contact[i] = _bioTac->getStatus(i);
                    _fingerMsg.Pdc[i] = _bioTac->getPdc(i);

                    Eigen::Vector3d _force = Eigen::Vector3d::Zero();
                    Eigen::Vector3d _normal = Eigen::Vector3d::Zero(); 
                    for (size_t j = 0; j < 3; j++)
                    {
                    _fingerMsg.Force[3*i+j] =_force[j];
                    _fingerMsg.nVec[3*i+j] =_normal[j];
                    }
                }
                _fingertipPublisher.publish(_fingerMsg);
                //logData
            
            }
            _mutex.unlock();
            ros::spinOnce();
            _loopRate.sleep();
        }
        ros::spinOnce();
        _loopRate.sleep();
        // _outputFile.close();
        ros::shutdown();
    }

  protected:
    double _dt;
    std::mutex _mutex;

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    ros::Subscriber _subBioTac;  
    ros::Publisher _plotPublisher;
    ros::Publisher _fingertipPublisher;

    std_msgs::Float64MultiArray _plotVar;
    biotac_ros::fingerMsg _fingerMsg;
    bool _stop;     // Check for CTRL+C
    std::shared_ptr<biotac::BiotacSensor> _bioTac;

  private:
    void updateBioTac(const std_msgs::String & msg){
        Eigen::VectorXd _bioTacData = Eigen::VectorXd::Zero(3*54);
        // std::cout << "biotac info" << msg << std::endl<< std::endl;
        std::vector<double> _data;
        std::stringstream ss(msg.data);
        while (ss.good()) { 
            std::string substr; 
            std::getline(ss, substr, ','); 
            _data.push_back(std::stod(substr)); 
        }
        if (_data.size() == (1+_bioTacData.size())){
            for (size_t i = 0; i < _bioTacData.size(); i++){
                _bioTacData[i] = _data[i+1]; 
            }
        }
        _bioTac->updateEntity(_bioTacData);
    }

};


int main (int argc, char **argv)
{
    float frequency = 100.0f; //200.0f;

    ros::init(argc,argv, "hand_master");
    ros::NodeHandle n;
   
    std::shared_ptr<BiotacRosMaster> BioTacMaster = std::make_shared<BiotacRosMaster>(n,frequency);

    if (!BioTacMaster->init()){
        return -1;
    }
    else{
        BioTacMaster->run();
    }
    return 0;
}