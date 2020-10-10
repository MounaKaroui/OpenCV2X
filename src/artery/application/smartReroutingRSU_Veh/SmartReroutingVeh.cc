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

#include <artery/application/smartReroutingRSU_Veh/SmartReroutingVeh.h>
#include "inet/common/ModuleAccess.h"
#include <iostream>
#include <algorithm>
#include <omnetpp/cpacket.h>
#include <vector>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "artery/utility/Channel.h"

namespace artery {
Define_Module(SmartReroutingVeh);

namespace si = boost::units::si;


simsignal_t SmartReroutingVeh::rcvdPkSignal = registerSignal("rcvdPk");
simsignal_t SmartReroutingVeh::routedVehSignal=registerSignal("routedVeh");



SmartReroutingVeh::~SmartReroutingVeh()
{


}



SmartReroutingVeh::SmartReroutingVeh()
{




}



void SmartReroutingVeh::initialize()
{
    ItsG5Service::initialize();
    m_self_msg = new cMessage("smart rerouting");
    vehicle=getFacilities().get_mutable_ptr<traci::VehicleController>();
    const auto jitter = uniform(SimTime(0, SIMTIME_MS),uniform(0,1));
    scheduleAt(simTime() + jitter + uniform(0,1), m_self_msg);

}


int SmartReroutingVeh::extractNumber(std::string input)
{
    size_t i = 0;
    for ( ; i < input.length(); i++ ){ if ( isdigit(input[i])) break; }
    // remove the first chars, which aren't digits
    input = input.substr(i, input.length() - i );
    // convert the remaining text to an integer
    int id = atoi(input.c_str());
    return id;
}


void SmartReroutingVeh::takeActions(TrafficInformation* trafficInfos)
{
    std::string vehicleId=vehicle->getVehicleId();
    if(trafficInfos->getEmergency())
    {
         double speed=trafficInfos->getRecommendedSpeed();
         vehicle->setSpeed(speed*si::meter_per_second);

         // if possible change the route else slow down the speed
         // TODO add rerouting function that avoid the accident zone
         //std::string route=trafficInfos->getAccidentZone();
         //vehicle->getLiteAPI().vehicle().getRoute(vehicleId);
    }
}

void SmartReroutingVeh::indicate(const btp::DataIndication& ind, cPacket* packet)
{
    Enter_Method("indicate");
    TrafficInformation* trafficInfo=dynamic_cast<TrafficInformation*>(packet);

    // if(trafficInfo->getSenderId()=="RSU")
   // {
        takeActions(trafficInfo);
        emit(rcvdPkSignal, trafficInfo);
   //}
}


void SmartReroutingVeh::handleMessage(cMessage *msg)
{
    Enter_Method("handleMessage");
    if (msg == m_self_msg) {
        EV_INFO << "self message\n";
    }
}


void SmartReroutingVeh::finish()
{
    // you could record some scalars at this point
    ItsG5Service::finish();
}


} //namespace
