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

class SmartReroutingRSU : public  ItsG5Service
{

  public:
    SmartReroutingRSU();
    ~SmartReroutingRSU();
	bool g5=false;
	bool mode4=false;
    int messageSize;

    static simsignal_t sentPkSignal;

    //static simsignal_t sentG5_PkSignal;
    //static simsignal_t sentMode4_PkSignal;

    double slowDownSpeed;
    std::string damagedVehId;
    std::string accidentZone;
  protected:
    void buildTrafficInfos(TrafficInformation*);
    void trigger() override;
    //McdaDecider* getDecider();

    virtual void handleMessage(cMessage *msg) override;
    void initialize()override ;
    void finish()override ;
  private:
    cMessage* m_self_msg;

};

} //namespace

#endif
