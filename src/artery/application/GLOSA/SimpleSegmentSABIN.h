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

#ifndef __ARTERY_ARTERY_IDE_SIMPLESEGMENTSABIN_H_
#define __ARTERY_ARTERY_IDE_SIMPLESEGMENTSABIN_H_

#include <artery/application/GLOSA/Glosa.h>
#include <omnetpp.h>
#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"
#include <math.h>
using namespace omnetpp;

/**
 * TODO - Generated class
 */

namespace artery {
class SimpleSegmentSABIN : public ItsG5Service
{

public:

    cOutVector totalSpeed;
    traci::VehicleController* vehicle;


    double arrivalTimeS1;
    double arrivalTimeS2;
    double arrivalTimeS3;
    double arrivalTimeS4;
    double arrivalTimeS5;

    double xlimit1;
    double xlimit2;
    double xlimit3;
    double xlimit4;
    double xlimit5;

    static simsignal_t rcvdPkSignal;
    static simsignal_t fuelSignal;
    static simsignal_t wtSignal;



    double speedDecision(GlosaPacket*,Glosa* , double , double, double );
    void calculateStats();
    void speedDecider(GlosaPacket*,Glosa* ,double , double , double );
    double getTravelDistance(GlosaPacket* packet);
    double verifSpeed(double speed, Glosa* glosa);
 protected:
    virtual void   indicate(const vanetza::btp::DataIndication&,  cPacket* packet) override;
    virtual void initialize() override;
    void finish() override;

};
}
#endif
