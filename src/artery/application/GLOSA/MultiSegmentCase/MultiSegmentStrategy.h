//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ARTERY_ARTERY_IDE_MULTISEGMENTSTRATEGY_H_
#define __ARTERY_ARTERY_IDE_MULTISEGMENTSTRATEGY_H_

#include <omnetpp.h>

#include <artery/application/GLOSA/Glosa.h>

#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>


#include <iostream>
#include "boost/icl/interval.hpp"
#include <boost/icl/interval_set.hpp>

using namespace omnetpp;

/**
 * TODO - Generated class
 */

//using namespace boost::icl;
//using namespace boost::numeric;
using namespace std;
class MultiSegmentStrategy : public ItsG5Service
{


public:

    CaObject* message ;
    double speedFromCAM;
    bool stoppedVeh;
    int segmentNumber;
    bool segmentOne;
    bool segmentTwo;
    bool segmentThree;
    bool segmentFour;
    bool segmentFive;

    simsignal_t  receivedCAMSignal;
    simsignal_t  receivedGlosaSignal;


    cOutVector rcvGlosaPackets;
    cOutVector stoppedVehicleNumbers;
    cOutVector totalSpeed;
    cOutVector fuelConsumption;
    cOutVector lastStep;

    cOutVector  totalRcvPacketsFromRSU;
    cOutVector passage1;
    cOutVector passage2;
    cOutVector passage3;
    cOutVector vehPositionVec;
    cOutVector v1_inter;

    double r0=0;
    double r1=0;
    double r2=0;
    cOutVector stoppageTime;
    double stopTime;
    cOutVector  receivedFromRSU0;
    cOutVector  receivedFromRSU1;
    cOutVector  receivedFromRSU2;

    cOutVector receptionTime;
    traci::TraCIPosition lastStoppedVehiclePosition;
    long rcvCAM;
    long rcvGlosa;
    long stoppedVehicleNumber;

    cOutVector speedFromCAM_vec;

    cOutVector speed_op1;
    cOutVector speed_op2;
    cOutVector speed_op3;
    cOutVector speed_op4;
    cOutVector speed_op5;

    cOutVector speed2_op1;
    cOutVector speed2_op2;
    cOutVector speed2_op3;
    cOutVector speed2_op4;
    cOutVector speed2_op5;
    cOutVector tp_vec;

    cOutVector  loweri1;
    cOutVector loweri2;
    cOutVector upperi1;
    cOutVector upperi2;

    cOutVector d0;
    cOutVector d1;
    cOutVector d2;
    cOutVector verifyInter;
    struct Interval {
        int start;
        int end;
    };
    traci::VehicleController* vehicle;
    std::map<int,MultiSegmentSABIN::speed> allSpeeds;
    std::map<int,MultiSegmentSABIN::trafficLightData> tlsdat;

    std::vector<double> t_min;
    std::vector<double> t_max;

   // void selectSpeed(Glosa* glosa,MultiSegmentSABIN::trafficLightData tls,int id,double v0, double t0, double deltaT);

    boost::icl::continuous_interval<double> getIntersection(boost::icl::continuous_interval<double> i1,boost::icl::continuous_interval<double> i2);
    void extractCamData();
    double calculateDistance(MultiSegmentSABIN::tlsPos tlsPosition, traci::TraCIPosition vehPosition);
    //void resolveSpeedWithDijkestra(int detectedSegmentNumber);
    void adaptSpeedToStoppedVehicle(Glosa* glosa, MultiSegmentSABIN::trafficLightData tls ,int id,double t0, double v0, double deltaT);
    void writeStatistics(Glosa* glosa,GlosaPacket* glosaPacket);


    void verifySpeed(Glosa* glosa,MultiSegmentSABIN::trafficLightData tls,int id, double v0, double t0,double deltaT);

    void calculateSpeedsForSegments(Glosa* glosa,int id, MultiSegmentSABIN::trafficLightData tls, double t0, double v0, double deltaT);
    void speedsDecider(Glosa* glosa, bool passage, double t0, double v0);
    double checkSpeed(double speed, Glosa* glosa);
    double adaptSpeed(Glosa* glosa, MultiSegmentSABIN::trafficLightData tls);
    bool DetermineCommonSpeed();

    void speedDecision1(Glosa* glosa, double t0, double v0);
    MultiSegmentSABIN::trafficLightData getTlsData(Glosa* glosa,GlosaPacket* packet, int i);
    bool trafficLightExist(GlosaPacket* glosaPacket);
    double calculateSumTravelTime(Glosa* glosa, double t0, double v0);

 protected:
    virtual void   receiveSignal(cComponent*, omnetpp::simsignal_t, cObject* obj, cObject*) override;
    virtual void   indicate(const vanetza::btp::DataIndication&,  cPacket* packet) override;

   virtual void initialize() override;
   void finish() override;
   virtual void handleMessage(cMessage *msg) override;

 private:
   cMessage* m_self_msg;

};

#endif
