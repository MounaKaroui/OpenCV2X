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

#ifndef __ARTERY_ARTERY_IDE_CBRAPPTEST_H_
#define __ARTERY_ARTERY_IDE_CBRAPPTEST_H_

#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include "apps/alert/AlertPacket_m.h"
#include "artery/traci/VehicleController.h"
#include "traci/Position.h"
#include "artery/application/NetworkInterface.h"


using namespace vanetza;
using namespace omnetpp;

namespace artery {

/**
 * TODO - Generated class
 */
class CbrAppTest : public  ItsG5Service
{

public:

               CbrAppTest();
              ~CbrAppTest();
               int extractNumber(std::string input);

               double recordingTime;
               //static simsignal_t delaySignal;
               static simsignal_t sentPkSignal;

               static simsignal_t rcvdPkSignal;

               static simsignal_t cbrSignal;
               static simsignal_t rcvdIdSignal;

               static simsignal_t sirSignal;

               static simsignal_t pirSignal;

               static simsignal_t distanceSignal;

               void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
               void trigger() override;

               int interface;

               void prepareIndication();
               void prepareMcoIndication();
               bool getId();

               AlertPacket* packet;
               btp::DataRequestB req;
               bool withMco;

               std::vector<long> id_Vec;


               std::set<int> id_list;

               double rcvCount=0;




               int receiveFromId;
               cOutVector delay;
               double delayItsG5;



               int messageSize;
               double sentMsg=0;
               double rcvMsg=0;
               traci::VehicleController* vehicle;
               int senderId;


               double numVeh;
               cOutVector transVeh;

               double positionX;
               double positionY;
               int sendNodeId;

               double camDelay=0;




      protected:
               void initialize()override ;
               void finish()override ;
               void handleMessage(omnetpp::cMessage*) override;
               void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
               void receiveSignal(cComponent*, simsignal_t signal, double value, cObject*) override;

      private:
             omnetpp::cMessage* m_self_msg;



};

} //namespace

#endif
