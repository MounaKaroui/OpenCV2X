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

#ifndef __ARTERY_ARTERY_IDE_MULTISEGMENTAPPROACH_H_
#define __ARTERY_ARTERY_IDE_MULTISEGMENTAPPROACH_H_


#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"
#include "SimpleSegmentSABIN.h"
#include "boost/icl/interval.hpp"
#include <boost/icl/interval_set.hpp>


using namespace omnetpp;

/**
 * TODO - Generated class
 */
class MultiSegmentApproach : public ItsG5Service
{

  public:

   cOutVector totalSpeed;
   cOutVector fuel;
   cOutVector commonSpeedVector;
   cOutVector commonSpeed1;
   cOutVector cmin;

   cOutVector travelDistanceVec;
   cOutVector distanceVec;
   cOutVector fuelLeader;
   cOutVector vehTravelDist;

   cOutVector dtl1;
   cOutVector dist1;
   cOutVector dist2;
   cOutVector dist3;
   cOutVector dist4;
   cOutVector dist5;
   cOutVector travelDist;


   double xlimit1;
   double xlimit2;
   double xlimit3;
   double xlimit4;
   double xlimit5;



   std::vector<double> intersect;
   bool intersection=false;
   bool passage=false;
   bool passage1=false;
   bool passage2=false;
   bool passage3=false;
   bool passage4=false;
   bool passage5=false;


   cOutVector speed1s1;
   cOutVector speed2s1;

   cOutVector speed1s2;
   cOutVector speed2s2;

   cOutVector speed1s3;
   cOutVector speed2s3;

   cOutVector speed1s4;
   cOutVector speed2s4;

   cOutVector speed1s5;
   cOutVector speed2s5;

   double commonSpeed;
   double speed;
   double d1=0;
   double d2=0;
   double d3=0;
   double d4=0;
   double d5=0;
   std::map<int,double> speed_map1;
   std::map<int,double> speed_map2;
   boost::icl::continuous_interval<double> inter;
   traci::VehicleController* vehicle;
   cOutVector arrivalTime;


   double getIntersection(boost::icl::continuous_interval<double> i1,boost::icl::continuous_interval<double> i2);
   void shiftCounter(int segmentNumber, int counter,GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT);
   void verifyPassage();

   void executeSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss);
   void determineSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss);
   void getTravelDistance(GlosaPacket* glosaPacket);
   double calculateArrivalTime(double t0, double v0,double d, double vs, Glosa* glosa);
   double determineV2(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT,double d);
   double determineV1(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT,double d);
   void verifSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss, double d);
   void  calculateAllSpeeds(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss);
   void calculateStats();


  protected:
    virtual void initialize() override;
    virtual void   indicate(const vanetza::btp::DataIndication&,  cPacket* packet) override;
    void finish() override;
};

#endif
