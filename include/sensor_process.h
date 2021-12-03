
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

#ifndef __BIOTAC_PROCESS_H__
#define  __BIOTAC_PROCESS_H__
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <vector>
namespace biotac{

    struct fingertip{
        Eigen::VectorXd El;
        Eigen::VectorXd El_offset;

        Eigen::VectorXd Pac;
        double Pdc, Tdc;
        double dPdc;
        double P_offset;
        bool active;
        bool contact;
    };
    class BiotacSensor
    {
    private:
        bool init();
        bool _isOk = false;
        bool _isFirst = true;
        size_t counter = 0;
        const size_t max_count = 100;
        double filter_gain = 0.4;
        double _dt = 0.005;

        
        Eigen::Vector3d plotVar = Eigen::Vector3d::Zero();
        biotac::fingertip addEntity(); //todo
    public:
        BiotacSensor();
        ~BiotacSensor(){};
        void updateEntity(const Eigen::VectorXd& data); //todo
        biotac::fingertip getEntity(const size_t& index);
        Eigen::VectorXd getElec(const size_t& index);
        double getPdc(const size_t& index);
        bool getStatus(const size_t& index);

        Eigen::Vector3d getPlotVar();
        bool isOk();
        std::vector<fingertip> fingertips;
    };

}

#endif
