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

#include <artery/application/GLOSA/Glosa.h>
#include "GlosaUseCaseRSU.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/btp/ports.hpp>
#include "artery/application/CaService.h"
#include "artery/traci/VehicleController.h"

#include "artery/utility/GlosaPacket_m.h"

#include <string>
#include <veins/base/modules/BaseMobility.h>

#include "traci/Core.h"
#include "traci/LiteAPI.h"
#include "traci/Position.h"
#include <inet/common/ModuleAccess.h>
#include <inet/mobility/contract/IMobility.h>

#include "artery/networking/StationaryPositionProvider.h"


using namespace omnetpp;
using namespace vanetza;


namespace artery {

Define_Module(GlosaUseCaseRSU);

simsignal_t GlosaUseCaseRSU::sentPkSignal = registerSignal("sentPk");



GlosaUseCaseRSU::GlosaUseCaseRSU(){


}


void GlosaUseCaseRSU::initialize()
{

    ItsG5Service::initialize();


    numRSU= getAncestorPar("numRoadSideUnits");

    packetSize=par("size").intValue();
    m_self_msg = new cMessage("GlosaUseCaseRSU Service");


    numRSU=0;





    scheduleAt(simTime()+uniform(0,0.5), m_self_msg);
}



int GlosaUseCaseRSU::extractNumber(std::string input)
{
    size_t i = 0;
    for ( ; i < input.length(); i++ ){ if ( isdigit(input[i]) ) break; }
    // remove the first chars, which aren't digits
    input = input.substr(i, input.length() - i );
    // convert the remaining text to an integer
    int id = atoi(input.c_str());
    return id;
}


traci::LiteAPI& GlosaUseCaseRSU::returnApi()
{
        int i=getRSUIndex();
        cModule* host=inet::getContainingNode(this);
        std::string module=host->getFullName();
        std::string modName=module+".vanetza[0].position";
        cModule* position=host->getModuleByPath(modName.c_str());
        StationaryPositionProvider* posProvider=dynamic_cast<StationaryPositionProvider*>(position);
        rsuPosition=posProvider->getInitialPosition();
        auto traci = inet::getModuleFromPar<traci::Core>(posProvider->par("traciCoreModule"), posProvider);
        return traci->getLiteAPI();

}


void GlosaUseCaseRSU::handleMessage(cMessage* msg)
{

    Enter_Method("handleMessage");
    if (msg == m_self_msg) {
        scheduleAt(simTime()+uniform(0,0.5), m_self_msg);
        EV_INFO << "self message\n";
    }
}

void GlosaUseCaseRSU::finish()
{
    cancelAndDelete(m_self_msg);
    ItsG5Service::finish();
}





void  GlosaUseCaseRSU::setTrafficLightState(GlosaPacket* packet)
{
    traci::LiteAPI& api=returnApi();
    double   t0=api.simulation().getCurrentTime()*0.001;
    std::vector<std::string> tlsId=api.trafficlights().getIDList();
    std::string state;

    if(theMostAdequateTls!=""){
        state=api.trafficlights().getRedYellowGreenState(theMostAdequateTls);
    }
    double redTime=30;
    double greenTime=25;
    double t_i,t_f;

    int tlsIndex=0;
    if(theMostAdequateTls!="")
    {
        if(state[tlsIndex]=='r')
        {
            t_i=(api.trafficlights().getNextSwitch(theMostAdequateTls))*0.001;
            t_f=t_i+greenTime;
        }

        if(state[tlsIndex]=='G')
        {

            t_f=(api.trafficlights().getNextSwitch(theMostAdequateTls))*0.001;

        }

        if(state[tlsIndex]=='y'){
            t_i=(api.trafficlights().getNextSwitch(theMostAdequateTls))*0.001+redTime;
            t_f=t_i+greenTime;
        }


    }
    packet->setStartGreenTime(t_i);
    packet->setEndGreenTime(t_f);
}

void GlosaUseCaseRSU::setTlsID(GlosaPacket* packet)
{

    traci::LiteAPI& api=returnApi();
    std::vector<std::string> tlsId=api.trafficlights().getIDList();
    int s=tlsId.size();
    double positionX[s];
    double positionY[s];

    for(int i=0;i<s;i++)
    {
        if(tlsId[i]!=""){
            positionX[i]=api.junction().getPosition(tlsId[i]).x;
            positionY[i]=api.junction().getPosition(tlsId[i]).y;
        } }




    if(numRSU==1)
    {
        theMostAdequateTls=tlsId[0];
    }
    else
    {

        for(int j=0;j<s;j++){
            int i=0;
            while (i<s){
                if((positionX[j]==rsuPosition.x/boost::units::si::meter))
                {
                    theMostAdequateTls=tlsId[j];

                }
                i++;
            }
        }
    }

    packet->setTlsId(theMostAdequateTls.c_str());

}




int GlosaUseCaseRSU::getRSUIndex()
{
    cModule* host=inet::getContainingNode(this);
    std::string module=host->getFullName();
    int index=extractNumber(module);
    return index;
}

void GlosaUseCaseRSU::encapsulateData(GlosaPacket* packet)
{
    int index=getRSUIndex();
    setTlsID(packet);
    setGeoData(packet);
    setTrafficLightState(packet);
    traci::LiteAPI& api=returnApi();

    const traci::Boundary boundary {api.simulation().getNetBoundary() };
    traci::TraCIPosition rsuPos=traci::position_cast(boundary, Position { rsuPosition.x, rsuPosition.y });

    packet->setX(rsuPos.x);
    packet->setY(rsuPos.y);
    packet->setZ(rsuPos.z);
    packet->setSenderId(index);
    packet->setByteLength(packetSize);
    packet->setFlag(1);
}





void GlosaUseCaseRSU::setGeoData(GlosaPacket* packet)
{
    double dist1=par("tlsDist1").doubleValue();
    double dist2=par("tlsDist2").doubleValue();
    double dist3=par("tlsDist3").doubleValue();
    double dist4=par("tlsDist4").doubleValue();
    double dist5=par("tlsDist5").doubleValue();

    packet->setDistancetl1(dist1);
    packet->setDistancetl2(dist2);
    packet->setDistancetl3(dist3);
    packet->setDistancetl4(dist4);
    packet->setDistancetl5(dist5);
}
void GlosaUseCaseRSU::trigger()
{

    // BTP-B is used because we create a DataRequestB ; We can use BTP-A by DataRequestA;
    Enter_Method("trigger");
    btp::DataRequestB req;
    req.destination_port = host_cast<GlosaUseCaseRSU::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    GlosaPacket* packet=new GlosaPacket("GLOSA RSU Packet");
    encapsulateData(packet);
    request(req, packet);
    emit(sentPkSignal,packet);
}

}
