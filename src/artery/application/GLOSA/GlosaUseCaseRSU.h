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

#ifndef __ARTERY_ARTERY_IDE_GLOSAUSECASERSU_H_
#define __ARTERY_ARTERY_IDE_GLOSAUSECASERSU_H_

#include <artery/application/GLOSA/Glosa.h>
#include <omnetpp.h>

#include "artery/application/ItsG5Service.h"
#include <vanetza/units/velocity.hpp>
#include "traci/LiteAPI.h"
#include <vanetza/asn1/cam.hpp>
#include  <string>

using namespace omnetpp;

/**
 * TODO - Generated class
 */

namespace artery {
class GlosaUseCaseRSU : public ItsG5Service
{
public:

    GlosaUseCaseRSU();
    int numRSU;
    Position rsuPosition;
    int packetSize;
    static simsignal_t sentPkSignal;

    traci::LiteAPI& returnApi();
    void trigger() override;
    int getRSUIndex();
    std::string theMostAdequateTls;


    void setTrafficLightState(GlosaPacket* packet);
    void setTlsID(GlosaPacket* packet);
    void setGeoData(GlosaPacket* packet);
    void encapsulateData(GlosaPacket* packet);
    int extractNumber(std::string input);

protected:
    void initialize() override;
    void finish() override;
    void handleMessage(omnetpp::cMessage*) override;

private:
    omnetpp::cMessage* m_self_msg;


};}


#endif