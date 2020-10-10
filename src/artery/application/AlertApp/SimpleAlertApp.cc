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

#include "SimpleAlertApp.h"
#include "artery/traci/VehicleController.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211Radio.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "inet/common/misc/ThruputMeter.h"

using namespace omnetpp;
using namespace vanetza;

namespace artery {

Define_Module(SimpleAlertApp);


//static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");


simsignal_t SimpleAlertApp::rcvdPkSignal = registerSignal("rcvdPk");
simsignal_t SimpleAlertApp::sentPkSignal = registerSignal("sentPk");

SimpleAlertApp::SimpleAlertApp()
{


}

SimpleAlertApp::~SimpleAlertApp()
{


}

void SimpleAlertApp::initialize()
{
    ItsG5Service::initialize();
    messageSize=par("size").doubleValue();

    m_self_msg = new cMessage("Alert Service");
    const auto jitter = uniform(SimTime(0, SIMTIME_MS), 0.1);
    scheduleAt(simTime() + jitter, m_self_msg);

}


void SimpleAlertApp::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");
    if (msg == m_self_msg) {
        EV_INFO << "self message\n";
    }
}






void SimpleAlertApp::indicate(const btp::DataIndication& ind, cPacket* packet)
{

    TrafficInformation* trafficInfos=dynamic_cast<TrafficInformation*>(packet);
    if(trafficInfos->getEmergency())
    {
    emit(rcvdPkSignal, trafficInfos);
    }
    delete(trafficInfos);

}



int SimpleAlertApp::extractNumber(std::string input)
{
    size_t i = 0;
    for ( ; i < input.length(); i++ ){ if ( isdigit(input[i]) ) break; }
    // remove the first chars, which aren't digits
    input = input.substr(i, input.length() - i );
    // convert the remaining text to an integer
    int id = atoi(input.c_str());
    return id;
}



void SimpleAlertApp::trigger()
{
    Enter_Method("trigger");


    btp::DataRequestB req;
    req.destination_port = host_cast<SimpleAlertApp::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    TrafficInformation* trafficInfos=new TrafficInformation("emergencyCase");
    trafficInfos->setByteLength(messageSize);
    trafficInfos->setEmergency(true);

    request(req, trafficInfos);
    emit(sentPkSignal,trafficInfos);

}

void SimpleAlertApp::finish()
{
    // you could record some scalars at this point
    ItsG5Service::finish();
}

} //namespace

