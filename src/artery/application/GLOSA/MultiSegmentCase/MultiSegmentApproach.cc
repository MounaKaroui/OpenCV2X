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

#include "MultiSegmentApproach.h"

Define_Module(MultiSegmentApproach);

using namespace omnetpp;
using namespace vanetza;
namespace si = boost::units::si;

void MultiSegmentApproach::initialize()
{

  ItsG5Service::initialize();
  commonSpeedVector.setName("commonSpeed");
  totalSpeed.setName("totalSpeed");
  fuel.setName("fuelConsumption");
  commonSpeed1.setName("commonSpeed1");
  vehTravelDist.setName("VehtravelDis");
  commonSpeed=0;
  speed1s1.setName("v1s1");
  speed2s1.setName("v2s1");

  speed1s2.setName("v1s2");
  speed2s2.setName("v2s2");

  speed1s3.setName("v1s3");
  speed2s3.setName("v2s3");

  speed1s4.setName("v1s4");
  speed2s4.setName("v2s4");

  speed1s5.setName("v1s5");
  speed2s5.setName("v2s5");

  cmin.setName("min");
  travelDistanceVec.setName("travelDistance");
  distanceVec.setName("Distance");
  dtl1.setName("dtl1");

  dist1.setName("dist1");
  dist2.setName("dist2");
  dist3.setName("dist3");
  dist4.setName("dist4");
  dist5.setName("dist5");

  travelDist.setName("Distance");
  arrivalTime.setName("arrivalTime");
  fuelLeader.setName("FuelLeader");

  xlimit1=2465;
  xlimit2=2066;
  xlimit3=1473;
  xlimit4=932;
  xlimit5=532;

}


void MultiSegmentApproach::getTravelDistance(GlosaPacket* packet)
{
    double td=vehicle->getLiteAPI().vehicle().getDistance(vehicle->getVehicleId());
    vehTravelDist.record(td);
    double xveh1=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId()).x;

    if((packet->getSenderId()==0)&&((xveh1>xlimit1)))
    {
         d1=(abs(packet->getDistancetl1()-td));

         dist1.record(d1);
    }


    if((packet->getSenderId()==1)&&((xveh1>xlimit2)))
    {
        d2=(abs(packet->getDistancetl2()-td));
        dist2.record(d2);
    }


    if((packet->getSenderId()==2)&&(((xveh1>xlimit3))))
    {
        d3=(abs(packet->getDistancetl3()-td));
        dist3.record(d3);
    }

    if((packet->getSenderId()==3)&&(((xveh1>xlimit4))))
     {
         d4=(abs(packet->getDistancetl4()-td));
         dist4.record(d4);
     }


    if((packet->getSenderId()==4)&&((xveh1>xlimit5)))
     {
         d5=(abs(packet->getDistancetl5()-td));
         dist5.record(d5);
     }
}

double MultiSegmentApproach::determineV1(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT,double d)
{
    traci::TraCIPosition vehPosition;

    vehPosition=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId());

    double t1=glosaPacket->getStartGreenTime();
    double t2=glosaPacket->getEndGreenTime();
    glosa->calculateSpeedBorder(t1,t2,t0,d,v0,deltaT);


    double n=0;
    double advisorySpeed=0;

    while((glosa->boundarySpeeds.v1==-1)&&(n<3))
    {
         n++;
         glosa->calculateSpeedBorder(t1+n*glosa->cycleDuration, t2+n*glosa->cycleDuration,t0,d,v0,deltaT);
    }


    if((glosa->boundarySpeeds.v1==-1))
    {
        advisorySpeed=glosa->minSpeed;
    }

    if((glosa->boundarySpeeds.v1!=-1)&&(glosa->boundarySpeeds.v1!=0))
    {
        advisorySpeed=glosa->boundarySpeeds.v1;
    }

     return advisorySpeed;
}




double MultiSegmentApproach::determineV2(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT,double d)
{
    traci::TraCIPosition vehPosition;
    traci::TraCIPosition tlsPosition;
    vehPosition=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId());
    tlsPosition.x=glosaPacket->getX();
    tlsPosition.y=glosaPacket->getY();
    //double d =ss->calculateDistance(tlsPosition, vehPosition);
    double t1=glosaPacket->getStartGreenTime();
    double t2=glosaPacket->getEndGreenTime();
    glosa->calculateSpeedBorder(t1,t2,t0,d,v0,deltaT);

    double n=0;
    double advisorySpeed=0;

    while((glosa->boundarySpeeds.v2==-1)&&(n<3))
    {
         n++;
         glosa->calculateSpeedBorder(t1+n*glosa->cycleDuration, t2+n*glosa->cycleDuration,t0,d, v0,deltaT);
    }


    if((glosa->boundarySpeeds.v2==-1))
    {
        advisorySpeed=glosa->minSpeed;
    }

    if((glosa->boundarySpeeds.v2!=-1)&&(glosa->boundarySpeeds.v2!=0))
    {
        advisorySpeed=glosa->boundarySpeeds.v2;
    }

     return advisorySpeed;
}



void  MultiSegmentApproach::verifyPassage()
{
    std::vector<libsumo::TraCINextTLSData> vehicleTLSData=vehicle->getLiteAPI().vehicle().getNextTLS(vehicle->getVehicleId());
    int size=vehicleTLSData.size();

    if(size==0)
        {
    	passage=true;
        }

    double xveh=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId()).x;

    if((xveh>xlimit1))
    {
        passage1=true;
    }

    if((xveh<xlimit1)&&(xveh>xlimit2))
    {
    	passage2=true;

    }

    if((xveh<xlimit2)&&(xveh>xlimit3))
    {
    	passage3=true;
    }

    if((xveh<xlimit3)&&(xveh>xlimit4))
    {
            passage4=true;
    }

    if((xveh>xlimit5)&&(xveh<xlimit4))
       {
            passage5=true;
       }

}


void MultiSegmentApproach::calculateAllSpeeds(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss)
{

    double v1s1=0;
    double v1s2=0;
    double v1s3=0;
    double v1s4=0;
    double v1s5=0;

    double v2s1=0;
    double v2s2=0;
    double v2s3=0;
    double v2s4=0;
    double v2s5=0;

    double xveh=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId()).x;
    getTravelDistance(glosaPacket);
    if((glosaPacket->getSenderId()==0)&&(xveh>xlimit1))
    {

      v1s1=ss->speedDecision(glosaPacket, glosa, t0, v0, deltaT);
      v2s1=determineV2(glosaPacket, glosa, t0, v0, deltaT,d1);
      // value update
      if(v1s1!=0)
      {
          speed_map1.insert(std::make_pair(0,v1s1));
          speed_map1[0]=v1s1;
          speed1s1.record(v1s1);
      }
      if(v2s1!=0)
      {
      speed_map2.insert(std::make_pair(0,v2s1));
      speed_map2[0]=v2s1;
      speed2s1.record( speed_map2[0]);
      }
    }

    if((glosaPacket->getSenderId()==1) &&(xveh>xlimit2))
    {
        v1s2=determineV1(glosaPacket, glosa, t0, v0, deltaT,d2);
        v2s2=determineV2(glosaPacket, glosa, t0, v0, deltaT,d2);
        // value update
        if(v1s2!=0)
        {
        speed_map1.insert(std::make_pair(1,v1s2));
        speed_map1[1]=v1s2;
        speed1s2.record(speed_map1[1]);
        }

        if(v2s2!=0)
        {
        speed_map2.insert(std::make_pair(1,v2s2));
        speed_map2[1]=v2s2;
        speed2s2.record(speed_map2[1]);
        }

    }

    if((glosaPacket->getSenderId()==2)&&(xveh>=xlimit3))
    {
        v1s3=determineV1(glosaPacket, glosa, t0, v0, deltaT,d3);
        v2s3=determineV2(glosaPacket, glosa, t0, v0, deltaT,d3);


        if(v1s3!=0)
        {
            speed_map1.insert(std::make_pair(2,v1s3));
            speed_map1[2]=v1s3;
            speed1s3.record(speed_map1[2]);
        }
        if(v2s3!=0)
        {
            speed_map2.insert(std::make_pair(2,v2s3));
            speed_map2[2]=v2s3;
            speed2s3.record(speed_map2[2]);

        }}
        // value update
        if((glosaPacket->getSenderId()==3)&&(xveh>xlimit4))
           {
               v1s4=determineV1(glosaPacket, glosa, t0, v0, deltaT,d4);
               v2s4=determineV2(glosaPacket, glosa, t0, v0, deltaT,d4);


               if(v1s4!=0)
               {
                   speed_map1.insert(std::make_pair(3,v1s4));
                   speed_map1[3]=v1s4;
                   speed1s4.record(speed_map1[2]);
               }
               if(v2s4!=0)
               {
                   speed_map2.insert(std::make_pair(3,v2s4));
                   speed_map2[3]=v2s4;
                   speed2s4.record(speed_map2[2]);

               }}

               if((glosaPacket->getSenderId()==4)&&((xveh>xlimit5)))
                  {
                      v1s5=determineV1(glosaPacket, glosa, t0, v0, deltaT,d5);
                      v2s5=determineV2(glosaPacket, glosa, t0, v0, deltaT,d5);


                      if(v1s5!=0)
                      {
                          speed_map1.insert(std::make_pair(4,v1s5));
                          speed_map1[4]=v1s5;
                          speed1s5.record(speed_map1[2]);
                      }
                      if(v2s5!=0)
                      {
                          speed_map2.insert(std::make_pair(4,v2s5));
                          speed_map2[4]=v2s5;
                          speed2s5.record(speed_map2[2]);

                      }}

}






double  MultiSegmentApproach::getIntersection(boost::icl::continuous_interval<double> i1,boost::icl::continuous_interval<double> i2)
{
      boost::icl::continuous_interval<double> inter=boost::icl::continuous_interval<double>::closed(0,0);
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

      inter=boost::icl::continuous_interval<double>::closed(inter1,inter2);
      return  inter.lower();
}






void  MultiSegmentApproach::shiftCounter(int segmentNumber, int counter,GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT)
{
					boost::icl::continuous_interval<double> i1=boost::icl::continuous_interval<double>::closed(0,0);
					boost::icl::continuous_interval<double> i2=boost::icl::continuous_interval<double>::closed(0,0);
					std::vector<double> vecSpeed1;
	            	for(int i=counter;i<segmentNumber;i++)
	           	        {
	           	        	for(int j=i+1; j<=(segmentNumber-1);j++)
	           	        	{
	           	      		i1=boost::icl::continuous_interval<double>::closed(speed_map2.find(i)->second,speed_map1.find(i)->second);
	           	            i2=boost::icl::continuous_interval<double>::closed(speed_map2.find(j)->second,speed_map1.find(j)->second);
	           	            if((speed_map2.find(i)->second>speed_map1.find(i)->second)||(speed_map2.find(j)->second>speed_map1.find(j)->second))
	           	            {
	           	                	 i1=boost::icl::continuous_interval<double>::closed(speed_map1.find(i)->second,speed_map2.find(i)->second);
	           	                	 i2=boost::icl::continuous_interval<double>::closed(speed_map1.find(j)->second,speed_map2.find(j)->second);
	           	            }

	           	            intersection=boost::icl::intersects(i1, i2);
	           	            if((intersection==true))
	           	            {
	           	                   commonSpeed=getIntersection(i1,i2);
	           	                   return;

	           	            }
	           	        	}
	           	            }
}

void MultiSegmentApproach::executeSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss)
{

if((commonSpeed<glosa->minSpeed))
{
ss->speedDecider(glosaPacket, glosa, t0, v0, deltaT);
}
else
{
vehicle->setSpeed(commonSpeed*si::meter_per_second);
}
}



double MultiSegmentApproach::calculateArrivalTime(double t0, double v0,double d, double vs, Glosa* glosa)
{
    double t_arr;
    double dis=0;
    if(vs>v0)
    {
    	dis=(vs*vs-v0*v0)/(2*glosa->accel);
    }
    else
    {
    	dis=(v0*v0-vs*vs)/(2*glosa->decel);
    }
    if(d>dis)
    {
        t_arr=(2*glosa->accel*d+(vs-v0)*(vs-v0))/(2*glosa->accel*vs)+t0;
    }
    else if(d<dis)
    {
        t_arr=(2*glosa->decel*d+(vs-v0)*(vs-v0))/(2*glosa->decel*vs)+t0;
    }
    else
    {
        t_arr=(-v0+sqrt(v0*v0+glosa->accel*d))/(2*glosa->accel)+t0;
    }

     return t_arr;
}


void MultiSegmentApproach::verifSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss, double d)
{
	   	   	   	   	   	double tarr=calculateArrivalTime(t0,v0,d,commonSpeed,glosa);
                    	arrivalTime.record(tarr);
                       	if((tarr>glosaPacket->getStartGreenTime())&&(tarr<glosaPacket->getEndGreenTime()))
                       	{
                       		executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
                       	}
                       	else
                       	{
                       		ss->speedDecider(glosaPacket, glosa, t0, v0, deltaT);
                       	}
}

void MultiSegmentApproach::determineSpeed(GlosaPacket* glosaPacket,Glosa* glosa, double t0, double v0, double deltaT, SimpleSegmentSABIN* ss)
{

               calculateAllSpeeds(glosaPacket,glosa,t0,v0,deltaT,ss);
               int segmentNumber=5;
               double tarr=0;
               if(passage1)
        	   {
        	   shiftCounter(segmentNumber,0,glosaPacket, glosa, t0, v0, deltaT);
        	   executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
        	   //verifSpeed(glosaPacket, glosa, t0, v0, deltaT, ss, d1);
               }
               if(passage2)
        	   {

        		 shiftCounter(segmentNumber, 1,glosaPacket, glosa, t0, v0, deltaT);
        		 executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
        		 //verifSpeed(glosaPacket, glosa, t0, v0, deltaT, ss, d2);

        	   }
               if(passage3)
               {

                shiftCounter(segmentNumber, 2,glosaPacket, glosa, t0, v0, deltaT);
                executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
                //verifSpeed(glosaPacket, glosa, t0, v0, deltaT, ss, d3);

                }

               if(passage4)
               {
                 shiftCounter(segmentNumber,3,glosaPacket, glosa, t0, v0, deltaT);
                executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
                //verifSpeed(glosaPacket, glosa, t0, v0, deltaT, ss, d4);

                }

               if(passage5)
               {
            	 shiftCounter(segmentNumber,4,glosaPacket, glosa, t0, v0, deltaT);
            	 executeSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);
            	 //verifSpeed(glosaPacket, glosa, t0, v0, deltaT, ss, d5);

               }








}


void  MultiSegmentApproach::indicate(const vanetza::btp::DataIndication&,  cPacket* packet)
{

        double v0=0;
        double t0=0;
        Glosa* glosa=new Glosa();
        SimpleSegmentSABIN* ss=new SimpleSegmentSABIN();
        GlosaPacket* glosaPacket = dynamic_cast<GlosaPacket*>(packet);

       vehicle=getFacilities().get_mutable_ptr<traci::VehicleController>();
       glosa->accel=vehicle->getLiteAPI().vehicle().getAccel(vehicle->getVehicleId());
       glosa->decel=(-1)*vehicle->getLiteAPI().vehicle().getDecel(vehicle->getVehicleId());
       glosa->maxSpeed=vehicle->getLiteAPI().vehicle().getMaxSpeed(vehicle->getVehicleId());
       glosa->minSpeed=5.5;

       glosa->cycleDuration=60;
       glosa->vehicle=this->vehicle;
       ss->vehicle=this->vehicle;
       ss->xlimit1=xlimit1;
       ss->xlimit2=xlimit2;
       ss->xlimit3=xlimit3;
       ss->xlimit4=xlimit4;
       ss->xlimit5=xlimit5;

       v0=vehicle->getSpeed()/si::meter_per_second;
       t0=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
       double deltaT=5.5;

        if((glosaPacket->getFlag()==1))
        {

        	verifyPassage();

        	if(passage==true)
        	{
        		vehicle->setSpeed(vehicle->getMaxSpeed());
        	}
        	else
        	{
        	    determineSpeed(glosaPacket, glosa, t0, v0, deltaT, ss);


        	}
        }
        calculateStats();
}


void MultiSegmentApproach::calculateStats()
{
  speed=vehicle->getSpeed()/si::meter_per_second;
  totalSpeed.record(speed);
  commonSpeedVector.record(commonSpeed);
  double fc=vehicle->getLiteAPI().vehicle().getFuelConsumption(vehicle->getVehicleId());
  fuel.record(fc);

  //std::pair<std::string, double> leadId=vehicle->getLiteAPI().vehicle().getLeader(vehicle->getVehicleId(),0.5);
  //double fuellead=vehicle->getLiteAPI().vehicle().getFuelConsumption(leadId.first);
  //fuelLeader.record(fuellead);
}

void  MultiSegmentApproach::finish()
{

    ItsG5Service::finish();
}



