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

#ifndef __ARTERY_ARTERY_IDE_SIMPLEALERTTEST_H_
#define __ARTERY_ARTERY_IDE_SIMPLEALERTTEST_H_

#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include "artery/utility/TrafficInformation_m.h"

#include "artery/traci/VehicleController.h"
#include "traci/Position.h"
#include "artery/application/NetworkInterface.h"


using namespace omnetpp;
using namespace vanetza;

namespace artery {

/**
 * TODO - Generated class
 */

class SimpleAlertApp : public  ItsG5Service
{
    public:

            SimpleAlertApp();
           ~SimpleAlertApp();

            static simsignal_t sentPkSignal;
            static simsignal_t rcvdPkSignal;

            void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
            void trigger() override;
            int extractNumber(std::string input);


            btp::DataRequestB req;
            int messageSize;
            std::string senderVehId;


   protected:
            void initialize()override ;
            void finish()override ;
            void handleMessage(omnetpp::cMessage*) override;


   private:
          omnetpp::cMessage* m_self_msg;


};

} //namespace

#endif
