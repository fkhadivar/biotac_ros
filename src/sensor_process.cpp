
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
#include "sensor_process.h"

namespace biotac{
    BiotacSensor::BiotacSensor(){
        for(size_t i = 0; i < 3; i++) 
            fingertips.push_back(addEntity());
    }
    biotac::fingertip BiotacSensor::addEntity(){
        biotac::fingertip _fingertip;
        _fingertip.El = Eigen::VectorXd::Zero(19);
        _fingertip.El_offset = Eigen::VectorXd::Zero(19);

        _fingertip.Pac = Eigen::VectorXd::Zero(24);
        _fingertip.Pdc = 0.;
        _fingertip.dPdc = 0.;
        _fingertip.P_offset = 0.;
        _fingertip.Tdc = 0.;
        _fingertip.active = false;
        _fingertip.contact = false;

        return _fingertip;
    }

    void BiotacSensor::updateEntity(const Eigen::VectorXd& data){
        if(data.size() != (3*54)){
            std::cerr << "incorrect biotac input size" << std::endl;
            std::abort();
        }else if(_isFirst){
            counter ++;
            for (size_t index = 0; index < 3; index++){
                fingertips[index].P_offset += data[54*index + 2*24];
                
                Eigen::VectorXd _el = Eigen::VectorXd::Zero(24);
                for (size_t i = 0; i < 24; i++){
                    _el[i] = data[54*index + 2*i];
                }
                fingertips[index].El_offset += _el.segment(0,19);
                
                fingertips[index].Pdc += data[54*index + 2*24];
                fingertips[index].Tdc += data[54*index + 2*24+4];
                if(counter > max_count){
                    _isFirst = false;
                    fingertips[index].P_offset = fingertips[index].P_offset * 1/(max_count+1); //todo use sascha work and........
                    fingertips[index].Pdc = fingertips[index].Pdc * 1/(max_count+1); 
                    fingertips[index].Tdc = fingertips[index].Tdc * 1/(max_count+1);
                    fingertips[index].El_offset = fingertips[index].El_offset * 1/(max_count+1);
                }
            }
            // std::cout << "initializing biotac: "<< counter << std::endl << std::endl;  

        }else{
            _isOk = true;
            for (size_t index = 0; index < 3; index++){
                Eigen::VectorXd _el = Eigen::VectorXd::Zero(24);
                double _pdc , _tdc = 0.;
                for (size_t i = 0; i < 24; i++){
                    _el[i] = data[54*index + 2*i];
                    fingertips[index].Pac[i] = data[54*index + 2*i+1]; 
                }
                fingertips[index].contact = true;

                fingertips[index].dPdc = 0.2* (data[54*index + 2*24] - fingertips[index].Pdc)/_dt + (1-0.2)*fingertips[index].dPdc;
                if( fingertips[index].Pdc - fingertips[index].P_offset < 20 )
                    if(fabs(fingertips[index].dPdc) < 75){
                        fingertips[index].P_offset += 0.1* (fingertips[index].Pdc - fingertips[index].P_offset);
                        fingertips[index].El_offset+= 0.1*(_el.segment(0,19) - fingertips[index].El_offset);
                        fingertips[index].contact = false;
                    }
                // if(fingertips[index].dPdc < -2000)
                //     fingertips[index].contact = false;

                fingertips[index].Pdc = filter_gain * data[54*index + 2*24] + (1-filter_gain) * fingertips[index].Pdc;
                fingertips[index].Tdc = filter_gain * data[54*index + 2*24+4] + (1-filter_gain) * fingertips[index].Tdc;
                fingertips[index].El  = filter_gain * (_el.segment(0,19) - fingertips[index].El_offset) + (1-filter_gain) * fingertips[index].El;

                if (fingertips[index].El.norm() < 1){
                    fingertips[index].active = false;
                }else{
                    fingertips[index].active = true;
                }
            }
        }
    }

    biotac::fingertip BiotacSensor::getEntity(const size_t& index){
        return fingertips[index];
    }
    Eigen::VectorXd BiotacSensor::getElec(const size_t& index){
        return fingertips[index].El;
    }
    double BiotacSensor::getPdc(const size_t& index){
        return fingertips[index].Pdc - fingertips[index].P_offset;
    }

    bool BiotacSensor::getStatus(const size_t& index){
        return fingertips[index].contact;
    }
    
    bool BiotacSensor::isOk(){
        return _isOk;
    }
    Eigen::Vector3d BiotacSensor::getPlotVar(){
        return plotVar;
    }
}