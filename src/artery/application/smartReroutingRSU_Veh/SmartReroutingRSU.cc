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

#include <artery/application/smartReroutingRSU_Veh/SmartReroutingRSU.h>
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
Define_Module(SmartReroutingRSU);

simsignal_t SmartReroutingRSU::sentPkSignal = registerSignal("sentPk");



SmartReroutingRSU::~SmartReroutingRSU()
{


}



SmartReroutingRSU::SmartReroutingRSU()
{


}



void SmartReroutingRSU::initialize()
{
    ItsG5Service::initialize();
    slowDownSpeed=par("slowDownSpeed").doubleValue();
    damagedVehId=par("damagedVehId").stringValue();
    accidentZone=par("accidentZone").stringValue();
    messageSize=par("messageSize").intValue();
    m_self_msg = new cMessage("smart rerouting");
    const auto jitter = uniform(SimTime(0, SIMTIME_MS),uniform(0,1));
    scheduleAt(simTime() + jitter + uniform(0,1), m_self_msg);
}





void SmartReroutingRSU::handleMessage(cMessage *msg)
{
    Enter_Method("handleMessage");
    if (msg == m_self_msg)
    {
        EV_INFO << "self message\n";
    }
}




void SmartReroutingRSU::buildTrafficInfos(TrafficInformation* trafficInfos)
{
	trafficInfos->setTimestamp(simTime());
	trafficInfos->setByteLength(messageSize);
	trafficInfos->setDamagedVehId(damagedVehId.c_str());
	trafficInfos->setRecommendedSpeed(slowDownSpeed);
	trafficInfos->setAccidentZone(accidentZone.c_str());
	trafficInfos->setEmergency(true);
	trafficInfos->setSenderId("RSU");
}


void SmartReroutingRSU::trigger()
{
	Enter_Method("trigger");
	btp::DataRequestB req;
	req.destination_port = host_cast<SmartReroutingRSU::port_type>(getPortNumber());
	req.gn.transport_type = geonet::TransportType::SHB;
	req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
	req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
	TrafficInformation* trafficInfos=new TrafficInformation("emergencyCase");

	buildTrafficInfos(trafficInfos);
	request(req, trafficInfos);
	emit(sentPkSignal,trafficInfos);
}



void SmartReroutingRSU::finish()
{
    // you could record some scalars at this point
    ItsG5Service::finish();
}


} //namespace
