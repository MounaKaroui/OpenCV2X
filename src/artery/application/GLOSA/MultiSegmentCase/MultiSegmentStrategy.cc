
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

#include "MultiSegmentStrategy.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/btp/ports.hpp>
#include "artery/traci/MobilityBase.h"
#include "omnetpp/cwatch.h"
#include "artery/application/CaService.h"
#include "artery/messages/GlosaPacket_m.h"
#include <string>
Define_Module(MultiSegmentStrategy);
using namespace omnetpp;
using namespace vanetza;
namespace si = boost::units::si;

//static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
void MultiSegmentStrategy::initialize()
{

    ItsG5Service::initialize();

    receivedCAMSignal=registerSignal("receivedCAM");
    receivedGlosaSignal=registerSignal("rcvGlosa");
    rcvCAM=0;
    rcvGlosa=0;
    //subscribe(scSignalCamReceived);

    speedFromCAM_vec.setName("SpeedFromCAM");
    segmentNumber=par("segmentNumber");
    totalSpeed.setName("totalSpeed");
    fuelConsumption.setName("fuelPHEMLIGHT");
    receptionTime.setName("receptionTime");
    segmentOne=false;
    segmentTwo=false;
    segmentThree=false;

    speed_op1.setName("speed_op1");
    speed_op2.setName("speed_op2");
    speed_op3.setName("speed_op3");
    speed_op4.setName("speed_op4");
    speed_op5.setName("speed_op5");

    tp_vec.setName("passageTime");

    passage1.setName("passage1");
    passage2.setName("passage2");
    passage3.setName("passage3");
    totalRcvPacketsFromRSU.setName("totalRcvPacketsFromRSU");
    vehPositionVec.setName("vehiclePosition");

    receivedFromRSU0.setName("receivedFromRSU0");
    receivedFromRSU1.setName("receivedFromRSU1");
    receivedFromRSU2.setName("receivedFromRSU2");
    stoppageTime.setName("stopTime");
    lastStep.setName("lastStep");

    speed2_op1.setName("speed21");
    speed2_op2.setName("speed22");
    speed2_op3.setName("speed23");
    speed2_op4.setName("speed24");
    speed2_op5.setName("speed25");

    d0.setName("dtl1");
    d1.setName("dtl2");
    d2.setName("dtl3");
    v1_inter.setName("v1");
    loweri1.setName("loweri1");
    loweri2.setName("loweri2");

    upperi1.setName("upperi1");
    upperi2.setName("upperi2");
    verifyInter.setName("verifInter");
    for(int i=0; i<segmentNumber;i++)
        	{
        	allSpeeds.find(i)->second.v1=0;
        	allSpeeds.find(i)->second.v2=0;
        	}

}
double MultiSegmentStrategy::adaptSpeed(Glosa* glosa, MultiSegmentSABIN::trafficLightData tls)
{
	       // The question is how to find ro;
            // determine ro ?
	       //tc=ro*r/(1-ro); // ro is the rate of arrival divided by departure, r is the red time.
	       //glosa->calculateSpeedBorder(tls.startTime+tc, tls.endTime, t0, d, v0, deltaT);

}
void MultiSegmentStrategy::adaptSpeedToStoppedVehicle(Glosa* glosa, MultiSegmentSABIN::trafficLightData tls ,int id,double t0, double v0, double deltaT)
{


    traci::TraCIPosition vehPosition=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId());
    std::string laneId=vehicle->getLiteAPI().vehicle().getLaneID(vehicle->getVehicleId());

	double lastPosition=0;
	traci::TraCIPosition position;
    //std::vector<std::string> ctrlLanes=vehicle->getLiteAPI().trafficlights().getControlledLanes(tlsID);
    double delay= vehicle->getLiteAPI().vehicle().getWaitingTime(vehicle->getVehicleId());
    double count=0;

    if(delay>0)
    {
              count++;
       	      lastPosition= count*(vehicle->getLiteAPI().vehicle().getMinGap(vehicle->getVehicleId())+vehicle->getLiteAPI().vehicle().getLength(vehicle->getVehicleId()));
       	      position.x=lastPosition;
       	      lastStep.record(lastPosition);


       	      tls.distancetToTrafficLight-=lastPosition;
       	      //tls.startTime+=delay;
       	      calculateSpeedsForSegments(glosa,id,tls, t0, v0, deltaT+ delay);

          }



}


bool MultiSegmentStrategy::DetermineCommonSpeed()
{

	if((allSpeeds.find(0)->second.v1==allSpeeds.find(1)->second.v1)||(allSpeeds.find(1)->second.v1==allSpeeds.find(2)->second.v1))
	{

             return true;
	}
}

double MultiSegmentStrategy::calculateDistance(MultiSegmentSABIN::tlsPos tlsPosition, traci::TraCIPosition vehPosition)
{

    double d=0;
    double xDif= tlsPosition.x -  vehPosition.x;
    double yDif=tlsPosition.y  - vehPosition.y;
    d=sqrt(pow(xDif,2)+pow(yDif,2));

    return d;
}




MultiSegmentSABIN::trafficLightData MultiSegmentStrategy::getTlsData(Glosa* glosa,GlosaPacket* packet,int i)
{

    double d=0;
    MultiSegmentSABIN::tlsPos tlsPosition;
    traci::TraCIPosition vehPosition;
    MultiSegmentSABIN::trafficLightData data;
    std::vector<MultiSegmentSABIN::trafficLightData> tlsData;

    tlsPosition.x=packet->getX();
    tlsPosition.y=packet->getY();

    vehPosition=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId());
   // glosa->determineTheMostAdequateTls();

    d =calculateDistance(tlsPosition, vehPosition);
    data.tlsId=packet->getSenderId();

    if(data.tlsId==i)
    {
    	 data.distancetToTrafficLight=d;
    	 data.startTime=packet->getStartGreenTime();
    	 data.endTime=packet->getEndGreenTime();
         tlsdat.insert(std::make_pair(i,data));
         return data;
    }

}

void MultiSegmentStrategy::calculateSpeedsForSegments(Glosa* glosa,int id,MultiSegmentSABIN::trafficLightData tls, double t0, double v0, double deltaT)
{

	 MultiSegmentSABIN::speed speeds;
 	 speeds.v1=0;
 	 speeds.v2=0;
 	 	int n=0;


   glosa->calculateSpeedBorder(tls.startTime, tls.endTime, t0, tls.distancetToTrafficLight, v0, deltaT);

   while((glosa->boundarySpeeds.v1==-1)&&(n<3))
    {
                  n++;
                  glosa->calculateSpeedBorder(tls.startTime+n*glosa->cycleDuration, tls.endTime+n*glosa->cycleDuration, t0, tls.distancetToTrafficLight, v0,deltaT);
     }
    if(n>3 && glosa->boundarySpeeds.v1==-1)
    {
    glosa->boundarySpeeds.v1=glosa->minSpeed;
    }
   if((glosa->boundarySpeeds.v1!=-1)&&(glosa->boundarySpeeds.v1!=0))
   {
       speeds.v1=glosa->boundarySpeeds.v1;
   }

   if((glosa->boundarySpeeds.v2!=-1)&&(glosa->boundarySpeeds.v2!=0))
   {
       speeds.v2=glosa->boundarySpeeds.v2;
   }


    allSpeeds.insert(std::make_pair(id,speeds));

   speed_op1.record(allSpeeds.find(0)->second.v1*3.6);
   speed_op2.record(allSpeeds.find(1)->second.v1*3.6);
   speed_op3.record(allSpeeds.find(2)->second.v1*3.6);

   // speed_op4.record(allSpeeds.find(3)->second.v1);
   // speed_op5.record(allSpeeds.find(4)->second.v1);

    speed2_op1.record(allSpeeds.find(0)->second.v2*3.6);
    speed2_op2.record(allSpeeds.find(1)->second.v2*3.6);
    speed2_op3.record(allSpeeds.find(2)->second.v2*3.6);

    //speed2_op4.record(allSpeeds.find(3)->second.v2);
    //speed2_op5.record(allSpeeds.find(4)->second.v2);
}

bool MultiSegmentStrategy::trafficLightExist(GlosaPacket* glosaPacket)
{
   std::vector<libsumo::TraCINextTLSData> vehicleTLSData=vehicle->getLiteAPI().vehicle().getNextTLS(vehicle->getVehicleId());
   int size=vehicleTLSData.size();
   bool passage;

    if(size==0)
    {
      passage=true;
    }
    else
    {
        double xveh=vehicle->getPosition().x/boost::units::si::meter;
        if((glosaPacket->getSenderId()==0)&&(xveh>2064)){
            segmentOne=true;
        }
        if((glosaPacket->getSenderId()==1)&&((xveh<2064)&&(xveh>1473)))
        {
            segmentTwo=true;
            double p1=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
            passage1.record(p1);
        }
        if((glosaPacket->getSenderId()==2)&&((xveh>931)&&(xveh<1473)))
        {
            segmentThree=true;
            double p2=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
            passage2.record(p2);

        }

        passage=false;
    }

    return passage;
}

double MultiSegmentStrategy::checkSpeed(double speed, Glosa* glosa)
{
if(speed!=0)
{
    return speed;
}
else
{
  speed=glosa->minSpeed;
}
}

void MultiSegmentStrategy::speedDecision1(Glosa* glosa, double t0, double v0)
{

double speed;
if(segmentOne){
    speed=checkSpeed(allSpeeds.find(0)->second.v1, glosa);
    vehicle->setSpeed(speed*si::meter_per_second);
    }
if(segmentTwo)
{
     speed=checkSpeed(allSpeeds.find(1)->second.v1, glosa);
     vehicle->setSpeed(speed*si::meter_per_second);
}
if(segmentThree) {
       speed=checkSpeed(allSpeeds.find(2)->second.v1, glosa);
       vehicle->setSpeed(speed*si::meter_per_second);
}


}




double MultiSegmentStrategy::calculateSumTravelTime(Glosa* glosa, double t0, double v0)
{
           double total_minArr_Time=0;

           for(int i=0; i<segmentNumber;i++)
           {

               //total_minArr_Time+=glosa->calculateMinSafeArrivalTime(t0, v0, tlsdat.find(i)->second.distancetToTrafficLight);

           }
           return total_minArr_Time;

}



boost::icl::continuous_interval<double> MultiSegmentStrategy::getIntersection(boost::icl::continuous_interval<double> i1,boost::icl::continuous_interval<double> i2)
{
	 boost::icl::continuous_interval<double> inter;
	  double inter1=0;
	  double inter2=0;
	 if(upper(i1)==upper(i2))
		                {
		                inter1=upper(i1);
		                }
		                else
		                {
		                   inter1=std::min(upper(i1), upper(i2));
		                }

		                if(lower(i1)==lower(i2))
		                {
		                	inter2=lower(i1);
		                }
		                else
		                {
		                	inter2=std::max(lower(i1),lower(i2));
		                }
		                   loweri1.record(lower(i1));
		                   loweri2.record(lower(i2));

		                   upperi1.record(upper(i1));
						   upperi2.record(upper(i2));

						   inter=boost::icl::continuous_interval<double>::closed(inter1,inter2);
			return  inter;
}

void MultiSegmentStrategy::verifySpeed(Glosa* glosa,MultiSegmentSABIN::trafficLightData tls,int id,double v0, double t0, double deltaT)
{


    		adaptSpeedToStoppedVehicle(glosa, tls, id, t0, v0, deltaT);
	        double min=0;
	        std::vector<double> speed1;
	        std::vector<double> speed2;
	        bool intersection;
	        double v1=0;
	        //bool empty=false;
	        std::vector<double> speedInter;
	        bool segment;


	        for(int i=0; i<segmentNumber;i++)
	        {

	            speed1.push_back(allSpeeds.find(i)->second.v1);
	            speed2.push_back(allSpeeds.find(i)->second.v2);

	        }

	        boost::icl::interval_set<double> test_set;
	        boost::icl::continuous_interval<double> i1;
	        boost::icl::continuous_interval<double> i2;
	        boost::icl::continuous_interval<double> inter;
	        double key=0;

	       if (std::find(speed1.begin(), speed1.end(), key) != speed1.end())
	       {
	                	std::remove(speed1.begin(), speed1.end(), key);
	       }

	       if (std::find(speed2.begin(), speed2.end(), key) != speed2.end())
	       {
	                       	std::remove(speed2.begin(), speed2.end(), key);
	       }

	        // Determine interval intersection
    	    speedDecision1(glosa, t0, v0);

	        for(int i=0;i<(speed1.size());i++)
	        {
	        	for(int j=i+1; j<=(speed1.size()-1);j++)
	        	{

	      		if(((speed2.at(i)!=0)&&(speed1.at(i)!=0))&&((speed2.at(j)!=0)&&(speed1.at(j)!=0)))
	        		{
	      				i1=boost::icl::continuous_interval<double>::closed(speed2.at(i),speed1.at(i));
	                    i2=boost::icl::continuous_interval<double>::closed(speed2.at(j),speed1.at(j));
	                    if((speed2.at(i)>speed1.at(i))||(speed2.at(j)>speed1.at(j)))
	                    {
	                    	i1=boost::icl::continuous_interval<double>::closed(speed1.at(i),speed2.at(i));
	                    	i2=boost::icl::continuous_interval<double>::closed(speed1.at(j),speed2.at(j));
	                    }


                    intersection=boost::icl::intersects(i1, i2);
                    if(intersection==true)
	                {
	                   inter=getIntersection(i1, i2);
	                   v1=lower(inter);
	                   speedInter.push_back(v1);
	                   vehicle->setSpeed(lower(inter)*si::meter_per_second);
	                }
	                }
	      			v1_inter.record(lower(inter)*3.6);

	        	}
	            }


}

void MultiSegmentStrategy::speedsDecider(Glosa* glosa,bool passage, double t0, double v0)
{
    // the goal is to minimize the total stoppage time ; Assuming that we have 3 segments =>
    //it means that we have six speeds : Two speeds for each segment that enable driver  to  pass the traffic light.
    // The goal is to choose the best configuration  to pass all segments with a minimum stoppage time.
    // This  method will implement an algorithm that can choose the optimal speed
    //in order to pass  a set of consecutive traffic lights or to minimize the total stoppage time.

    //double speed=(allSegmentsSpeeds.at(0).v1+allSegmentsSpeeds.at(0).v2)/2;
    // vehicle->setSpeed(speed*si::meter_per_second);
    // calculation of the  speed for n cycle
    double speed=0;
    int index;
    std::vector<double> v;
    if(passage)
    {
         vehicle->setSpeed(vehicle->getMaxSpeed());
         double p3=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
                 passage3.record(p3);
    }
    else
    {
     speedDecision1(glosa, t0, v0);
    }
}








void MultiSegmentStrategy::indicate(const vanetza::btp::DataIndication&,  cPacket* packet)
{

    double v0=0;
    double t0=0;


    Glosa* glosa=new Glosa();
    vehicle=getFacilities().get_mutable_ptr<traci::VehicleController>();


    glosa->accel=vehicle->getLiteAPI().vehicle().getAccel(vehicle->getVehicleId());
    glosa->decel=-1*vehicle->getLiteAPI().vehicle().getDecel(vehicle->getVehicleId());
    glosa->maxSpeed=vehicle->getLiteAPI().vehicle().getMaxSpeed(vehicle->getVehicleId());
    glosa->minSpeed=5.5;
    glosa->cycleDuration=60;
    glosa->vehicle=vehicle;

    v0=vehicle->getSpeed()/si::meter_per_second;
    t0=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
    double deltaT=5.5;
    bool next;
    GlosaPacket* glosaPacket = dynamic_cast<GlosaPacket*>(packet);
    MultiSegmentSABIN::trafficLightData tls;


    if((glosaPacket->getFlag()==1))
    {

            int id=glosaPacket->getSenderId();
            tls=getTlsData(glosa,glosaPacket, id);
            calculateSpeedsForSegments(glosa,id,tls, t0, v0, deltaT);


            bool passage=trafficLightExist(glosaPacket);
            if(passage)
            {
                        vehicle->setSpeed(glosa->maxSpeed*si::meter_per_second);
                        double p3=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
                        passage3.record(p3);
            }
            else
            {

            			verifySpeed(glosa,tls,id, v0, t0,deltaT);

            }
            writeStatistics(glosa,glosaPacket);

    }

}






void MultiSegmentStrategy::writeStatistics(Glosa* glosa,GlosaPacket* glosaPacket)
{
    //speed_op1.record(allSpeeds.find(1)->second.v1*3.6);
    totalSpeed.record(vehicle->getSpeed()*3.6/si::meter_per_second);
    fuelConsumption.record(vehicle->getLiteAPI().vehicle().getFuelConsumption(vehicle->getVehicleId()));


      double rcvPacketFromRSU=0;
      rcvPacketFromRSU++;
      totalRcvPacketsFromRSU.record(rcvPacketFromRSU);
      //emit(rcvSignal,rcvPacketFromRSU);

      traci::TraCIPosition vehPos=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId());
      vehPositionVec.record(vehPos.x);
      //stopTime=vehicle->getLiteAPI().vehicle().getAccumulatedWaitingTime(vehicle->getVehicleId());
      //stoppageTime.record(stopTime);

      MultiSegmentSABIN::trafficLightData tls;
      if(glosaPacket->getSenderId()==0)
               {
                  r0++;
                  receivedFromRSU0.record(r0);
                  tls=getTlsData(glosa, glosaPacket, 0);
                  d0.record(tls.distancetToTrafficLight);
               }
               if(glosaPacket->getSenderId()==1)
               {

            	   tls=getTlsData(glosa, glosaPacket, 1);
            	   d1.record(tls.distancetToTrafficLight);
                   r1++;
                   receivedFromRSU1.record(r1);


               }
               if(glosaPacket->getSenderId()==2)
               {
                   r2++;
                   receivedFromRSU2.record(r2);
                   tls=getTlsData(glosa, glosaPacket, 2);
                   d2.record(tls.distancetToTrafficLight);

               }
}

void MultiSegmentStrategy::handleMessage(cMessage *msg)
{
    Enter_Method("handleMessage");
}


void  MultiSegmentStrategy::finish()
{
    //cancelAndDelete(m_self_msg);
      ItsG5Service::finish();
}




void MultiSegmentStrategy::extractCamData()
{
           vanetza::asn1::Cam camMsg;
           camMsg=message->asn1();
           CoopAwareness_t& camData = (*camMsg).cam;
           BasicContainer_t& basic = camData.camParameters.basicContainer;
           long  longitude= (basic.referencePosition.longitude/Longitude_oneMicrodegreeEast);
           long  latitude= (basic.referencePosition.latitude/Latitude_oneMicrodegreeNorth);
           HighFrequencyContainer_t& hfc = camData.camParameters.highFrequencyContainer;
           hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
           BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
           speedFromCAM= bvc.speed.speedValue/SpeedValue_oneCentimeterPerSec*pow(10,-2);
           speedFromCAM_vec.record(speedFromCAM);

}

void  MultiSegmentStrategy::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject*)
{

	    //if (signal == scSignalCamReceived)
	    //{
	    message = dynamic_cast<CaObject*>(obj);
	    extractCamData();
	    rcvCAM++;
	    emit(receivedCAMSignal, rcvCAM);
	   // }
}

