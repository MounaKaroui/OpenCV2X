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

#include "GlosaRSU.h"

#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/btp/ports.hpp>
#include "artery/application/CaService.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/GLOSA/GlosaPacket_m.h"

#include <string>


using namespace omnetpp;
using namespace vanetza;


namespace artery {

Define_Module(GlosaRSU);


 GlosaRSU::GlosaRSU(){

 }


void GlosaRSU::initialize()
{
        tlsIndex=0;
        distance=0;
        theMostAdequateTLSId="";
        ItsG5Service::initialize();
        m_self_msg = new cMessage("GlosaRSU Service");
        scheduleAt(simTime() + uniform(0,0.01), m_self_msg);
}


void GlosaRSU::indicate(const btp::DataIndication& ind, cPacket* packet)
{

        GlosaPacket* glosaPacket = dynamic_cast<GlosaPacket*>(packet);

       /* if ((glosaPacket->getFlag()==0))
        {

              //EV_INFO << "I am RSU !! I am receiving from vehicle !!";
              delete glosaPacket;
        }*/

}

void GlosaRSU::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");
    if (msg == m_self_msg) {
        EV_INFO << "self message\n";
    }
}
void GlosaRSU::finish()
{
    cancelAndDelete(m_self_msg);
    ItsG5Service::finish();
}

int   GlosaRSU::determineTheMostAdequateTls()
{
        std::vector<libsumo::TraCINextTLSData> vehicleTLSData=vehicle->getLiteAPI().vehicle().getNextTLS(vehicle->getVehicleId());
        traci::TraCIPosition tlsPosition;
        double d=0;
        double drivenDistance=0;

        std::vector<std::string> tlsID=vehicle->getLiteAPI().trafficlights().getIDList();

        int index=0;

        int size=vehicleTLSData.size();
        double distance[size];

        for(int i=0;i<vehicleTLSData.size();i++)
        {
         distance[i]=vehicleTLSData[i].dist;
        }

        smallest=distance[0];
        double vehDist=vehicle->getLiteAPI().vehicle().getDistance(vehicle->getVehicleId());


        Angle direction= vehicle->getHeading();

        for(int i=0;i<vehicleTLSData.size();i++)
        {

            if((vehicleTLSData[i].id==tlsID[i])||(distance[i]<smallest))
                   {
                   index=i;
                   smallest=vehicleTLSData[i].dist;
                   theMostAdequateTLSId=vehicleTLSData[i].id;
                   tlsIndex=vehicleTLSData[i].tlIndex;
                   }


        }
        return index;

}

void GlosaRSU::trigger()
{
           Enter_Method("trigger");
           btp::DataRequestB req;
           req.destination_port = host_cast<GlosaRSU::port_type>(getPortNumber());
           req.gn.transport_type = geonet::TransportType::SHB;
           req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
           req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

           GlosaPacket* packet=new GlosaPacket("RSU Packet");
           packet->setFlag(1);
           packet->setByteLength(41);
           request(req, packet);

}

void GlosaRSU::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject*)
{
    EV_INFO << "RSU "  << " entering receiveSignal method  of RSU...";


}

}
