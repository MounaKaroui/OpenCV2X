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

#ifndef __ARTERY_ARTERY_IDE_GlosaRSU_H_
#define __ARTERY_ARTERY_IDE_GlosaRSU_H_

#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include <vanetza/units/velocity.hpp>
#include "traci/API.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp/simtime.h>
#include "artery/traci/VehicleController.h"

using namespace omnetpp;

/**
 * TODO - Generated class
 */
namespace artery {
class GlosaRSU : public ItsG5Service
{
public:
    GlosaRSU();
    void trigger() override;
    int determineTheMostAdequateTls();
    std::vector<std::string> currentState;
    traci::VehicleController* vehicle;
    std::string theMostRelevantTLSId;
    double distance;
    double smallest;
    int tlsIndex;
    std::string theMostAdequateTLSId;
    void receiveSignal(cComponent* source, simsignal_t signal, cObject* obj, cObject*) override;
    void indicate(const vanetza::btp::DataIndication&, cPacket* glosaPacket) override;

protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        omnetpp::cMessage* m_self_msg;
};
}
#endif
