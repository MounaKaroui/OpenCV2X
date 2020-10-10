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
//@Mouna Karoui

#include <artery/application/GLOSA/Glosa.h>
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/btp/ports.hpp>
#include "artery/traci/MobilityBase.h"
#include "artery/application/CaService.h"
#include "artery/application/GLOSA/GlosaPacket_m.h"
#include <string>



using namespace omnetpp;
using namespace vanetza;
namespace si = boost::units::si;

namespace artery {

Define_Module(Glosa);

Glosa::Glosa()
{

}


void Glosa::initialize()
{


	receivedSignal=registerSignal("received");


    accel=0;
    decel=0;
    theMostAdequateTLSId="";
    startGreenTime=0;
    endGreenTime=0;

    receivedPacketFromRSU=0;
    boundarySpeeds.v1=0;
    boundarySpeeds.v2=0;

    minSpeed=5.55;
    cycleDuration=60;

    m_self_msg = new cMessage("Glosa Service");
	ItsG5Service::initialize();
}




double Glosa::calculateSpeed(double t,double d,double v0)
{

    double v=0;

              if((v0<d/t)&&(d>0))
              {
                  if((accel>2*(d-v0*t)/(t*t))&&(d>0))
                  {
                  v= accel*(t-sqrt(t*t-2*(d-v0*t)/accel))+v0;
                  if(v>0)
                  {
                      return v;
                  }
                  }
                  else
                  {
                      v=v0;
                  }
              }
              if ((v0>d/t)&&(d>0))
              {
                  if((decel<2*(d-v0*t)/(t*t))&&(d>0))
                  {
                  v= (decel)*(t-sqrt(t*t-2*(d-v0*t)/(decel)))+v0;
                  if(v>0)
                   {
                   return v;
                   }
                  }
                  else
                  {
                      v=v0;
                  }
             }


              return v;


}



double Glosa::calculateMinArrivalTime(double t0, double v0,double d)
{
    double t_arr;
    if(d>(maxSpeed*maxSpeed-v0*v0)/2*accel)
    {
        t_arr=(2*accel*d+(maxSpeed-v0)*(maxSpeed-v0))/(2*accel*maxSpeed)+t0;
    }else
    {
        t_arr=(-v0+sqrt(v0*v0+accel*d))/(2*accel)+t0;
    }

     return t_arr;
}


void Glosa::rerouteByQueue(std::string tlsId)
{
    traci::LiteAPI * m_api;
    std::vector<std::string> laneIds=m_api->trafficlights().getControlledLanes(tlsId);
    std::string vehId=vehicle->getVehicleId();

    int queueNum=0;
    for (int i=0;i<laneIds.size();i++)
    {
       queueNum+= m_api->lane().getLastStepHaltingNumber(laneIds[i]);
    }
    double wait=vehicle->getLiteAPI().vehicle().getWaitingTime(vehId);
    double totalWait=queueNum*wait;
    std::string laneId=vehicle->getLiteAPI().vehicle().getLaneID(vehId);

    double travelTime=vehicle->getLiteAPI().lane().getTraveltime(laneId);
    double weight=travelTime+totalWait;
    std::vector<std::string> edgeList=vehicle->getLiteAPI().vehicle().getRoute(vehId);
    std::string edgeId=vehicle->getLiteAPI().vehicle().getRoadID(vehId);

    for(int j=0; j<edgeList.size();j++)
    {
            if(edgeList[j]==edgeId)
            {
                //vehicle->getLiteAPI().edge().adaptTraveltime(edgeId, weight);
                vehicle->getLiteAPI().vehicle().rerouteTraveltime(vehId,weight);
            }
    }
}


void  Glosa::calculateSpeedBorder(double t1,double t2,double t0,double d,double v0, double deltaT)
{
      double v=0;
      double tarr_min=calculateMinArrivalTime(t0, v0, d);

      if(tarr_min>=(t2))
      {
          boundarySpeeds.v1=-1; //Exit
          boundarySpeeds.v2=-1;
    	  return;
          //rerouteByQueue(); --> TODO call reroute By queue in this scoope

      }

      v=calculateSpeed(t1-t0+deltaT,d,v0);
      if((tarr_min)<=(t1+deltaT))
      {
      if(v<minSpeed)
      {

          boundarySpeeds.v1=v0;

      }
      else
      {
          boundarySpeeds.v1=v;
      }
      }
      else
      {
          boundarySpeeds.v1=maxSpeed;
      }
     double v2=calculateSpeed(t2-t0,d, v0);
     if(v2<minSpeed)
       {
         boundarySpeeds.v2=minSpeed;
       }
       else
       {
           if(v2>maxSpeed)
           {
               boundarySpeeds.v2=v0;
           }
           else
           {
        		   boundarySpeeds.v2=v2;
           }
       }
}








void Glosa::indicate(const btp::DataIndication& ind, cPacket* packet)
{
              // gasoline driven light duty vehicle Euro norm 4
              GlosaPacket* glosaPacket = dynamic_cast<GlosaPacket*>(packet);


}



void Glosa::handleMessage(cMessage* msg)
{

    Enter_Method("handleMessage");
    if (msg == m_self_msg)
    {
        EV_INFO << "self message\n";

    }

}


void Glosa::trigger()
{



}



void Glosa::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject*)
{



}


void Glosa::finish()
{

       cancelAndDelete(m_self_msg);
       ItsG5Service::finish();

}
}

