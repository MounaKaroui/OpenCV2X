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

#ifndef __ARTERY_ARTERY_OMNETPP_SMARTREROUTING_H_
#define __ARTERY_ARTERY_OMNETPP_SMARTREROUTING_H_

#include <omnetpp.h>
#include <math.h>
#include "artery/utility/TrafficInformation_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/ItsG5Service.h"


using namespace omnetpp;
using namespace vanetza;

namespace artery {

class SmartReroutingVeh : public  ItsG5Service
{

  public:
    SmartReroutingVeh();
    ~SmartReroutingVeh();

    int routedVeh=0;
    int extractNumber(std::string input);

    static simsignal_t rcvdPkSignal;
    static simsignal_t routedVehSignal;


  protected:
    void takeActions(TrafficInformation*);
    virtual void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;


    virtual void handleMessage(cMessage *msg) override;
    virtual void initialize()override ;
    virtual void finish()override ;
  private:
    cMessage* m_self_msg;
    traci::VehicleController* vehicle;

};

} //namespace

#endif
