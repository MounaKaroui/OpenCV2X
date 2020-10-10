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

#include "CbrAppTest.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211Radio.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "artery/utility/Channel.h"

#include <set>
#include "artery/inet/InetRadioDriver.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include "artery/networking/Router.h"

#include "artery/application/CaService.h"


using namespace omnetpp;
using namespace vanetza;

namespace artery {

Define_Module(CbrAppTest);

static const simsignal_t scSignalCamReceived =cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent =cComponent::registerSignal("CamSent");


simsignal_t CbrAppTest::rcvdPkSignal = registerSignal("rcvdPk");

/////
simsignal_t CbrAppTest::rcvdIdSignal = registerSignal("rcvdId");
////

/////
simsignal_t CbrAppTest::sentPkSignal = registerSignal("sentPk");
////

//simsignal_t CbrAppTest::delaySignal = registerSignal("delay");

simsignal_t CbrAppTest::cbrSignal=registerSignal("cbr");

simsignal_t CbrAppTest::sirSignal=registerSignal("sir");

simsignal_t CbrAppTest::pirSignal=registerSignal("pir");



CbrAppTest::CbrAppTest()
{


}

CbrAppTest::~CbrAppTest()
{

}




void CbrAppTest::initialize()
{
    ItsG5Service::initialize();
    messageSize=par("size");
    m_self_msg = new cMessage("Alert Service");
    withMco=par("withMco").boolValue();

    recordingTime=par("recordingTime").doubleValue();
    senderId=0;
    delayItsG5=0;
    transVeh.setName("numVehTran");

    if(withMco)
    {
        interface=1;
    }
    else
    {
        interface=0;
    }

    //
   // cModule* host=inet::getContainingNode(this);
    //std::string module=host->getFullName();
    //std::string interfaceName=module+".radioDriver["+std::to_string(interface)+"]";
    //cModule* mod=host->getModuleByPath(interfaceName.c_str());
    //RadioDriverBase* radioDriver=dynamic_cast<RadioDriverBase*>(mod);
    //radioDriver->subscribe(RadioDriverBase::ChannelLoadSignal, this);

    //
    const auto jitter = uniform(SimTime(0, SIMTIME_MS),uniform(0,1));
    subscribe(scSignalCamReceived);
    subscribe(scSignalCamSent);
    //

    scheduleAt(simTime() + jitter + uniform(0,1), m_self_msg);
}


void CbrAppTest::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");
    if (msg == m_self_msg) {
        EV_INFO << "self message\n";
    }
}




int CbrAppTest::extractNumber(std::string input)
{
    size_t i = 0;
    for ( ; i < input.length(); i++ ){ if ( isdigit(input[i]) ) break; }
    // remove the first chars, which aren't digits
    input = input.substr(i, input.length() - i );
    // convert the remaining text to an integer
    int id = atoi(input.c_str());
    return id;
}




void CbrAppTest::indicate(const btp::DataIndication& ind, cPacket* packet)
{
    Enter_Method("indicate");
    AlertPacket* alertPacket=dynamic_cast<AlertPacket*>(packet);
    // &&(simTime().dbl()>=recordingTime)
    if((alertPacket->getByteLength()==messageSize))
    {
        ///double delay= (simTime()-alertPacket->getTimestamp()).dbl();

        //emit(delaySignal,delay);
        emit(rcvdPkSignal, alertPacket);

        //int num=extractNumber(alertPacket->getVehicleId());
        //emit(rcvdIdSignal,num);
    }
}





void CbrAppTest::prepareMcoIndication()
{
        static const vanetza::ItsAid example_its_aid = 16480;
        auto& mco = getFacilities().get_const<MultiChannelPolicy>();
        auto& networks = getFacilities().get_const<NetworkInterfaceTable>();

        for (auto channel : mco.allChannels(example_its_aid)) {
        auto network = networks.select(channel);
        if (network) {
        prepareIndication();
        // use same port number as configured for listening on this channel
        req.destination_port = host_cast(getPortNumber(channel));
        req.gn.transport_type = geonet::TransportType::SHB;
        req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
        req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
        req.gn.its_aid = example_its_aid;
        // send packet on specific network interface
        request(req, packet, network.get());
    } else {
        EV_ERROR << "No network interface available for channel " << channel << "\n";
    }
        }
}




void CbrAppTest::prepareIndication()
{

    req.destination_port = host_cast<CbrAppTest::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    packet=new AlertPacket("Alert test");
    packet->setTimestamp(simTime());

    cModule* host=inet::getContainingNode(this);
    std::string module=host->getFullName();
    //packet->setVehicleId(module.c_str());
    packet->setByteLength(messageSize);

}



template <typename ForwardIterator>
ForwardIterator remove_duplicates( ForwardIterator first, ForwardIterator last )
{
    auto new_last = first;

    for ( auto current = first; current != last; ++current )
    {
        if ( std::find( first, new_last, *current ) == new_last )
        {
            if ( new_last != current ) *new_last = *current;
            ++new_last;
        }
    }

    return new_last;
}

void  CbrAppTest::receiveSignal(cComponent* comp, simsignal_t signal, double value, cObject*)
{


	if (signal == RadioDriverBase::ChannelLoadSignal)
	{
		 cModule* module=dynamic_cast<cModule*>(comp);
		 std::string moduleName=module->getFullName();
         //std::string radioName="radioDriver["+std::to_string(interface)+ "]";
		 //if(moduleName==radioName)
		 //{
			  //ASSERT(value >= 0.0 && value <= 1.0);
			  //vanetza::dcc::ChannelLoad cl{value};
			  //if(simTime()>=recordingTime)
			  //{
			  //emit(cbrSignal,value*100);
			  //}
		 //}
	}
}



void  CbrAppTest::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal, omnetpp::cObject*, omnetpp::cObject*)
{
    if(signal==scSignalCamReceived)
    {
        emit(pirSignal,simTime().dbl());
    }

    if(signal==scSignalCamSent)
    {
        emit(sirSignal,simTime().dbl());
    }

}

void CbrAppTest::trigger()
{
    Enter_Method("trigger");

    if(withMco)
    {
        prepareMcoIndication();

    }
    else
    {

        prepareIndication();
        request(req, packet);

    }

    emit(sentPkSignal,packet);
}




void CbrAppTest::finish()
{
    // you could record some scalars at this point
    ItsG5Service::finish();


}

} //namespace
