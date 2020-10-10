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

#ifndef __ARTERY_ARTERY_IDE_ADAPTIVELIMERICDCCENTITY_H_
#define __ARTERY_ARTERY_IDE_ADAPTIVELIMERICDCCENTITY_H_

#include <omnetpp.h>
#include "artery/networking/DccEntityBase.h"
#include <vanetza/dcc/limeric.hpp>
#include <vanetza/dcc/limeric_transmit_rate_control.hpp>
#include <memory>
#include "inet/linklayer/ieee80211/mac/ratecontrol/RateControlBase.h"


using namespace omnetpp;
using namespace inet::ieee80211;

namespace artery {

/**
 * TODO - Generated class
 */
class AdaptiveLimericDccEntity :  public DccEntityBase
{
public:

    void finish() override;
    vanetza::dcc::TransmitRateThrottle* getTransmitRateThrottle() override;
    double updateTargetCbr(double bitrate);
    void getNeighboring();
    bool  withDDC;
    int interface;
    double  nodeInComm;

    double calculateIdleProbability(double tau);
    double calculateSucessTransmission(double tau);

    double updateTargetCbr2(double bitrate);

    static simsignal_t msgIntervalSignal;
    void calculateTargetCbr();

protected:
    void initializeTransmitRateControl() override;
    vanetza::dcc::TransmitRateControl* getTransmitRateControl() override;
    void onGlobalCbr(vanetza::dcc::ChannelLoad) override;

private:
    std::unique_ptr<vanetza::dcc::Limeric> mAlgorithm;
    std::unique_ptr<vanetza::dcc::LimericTransmitRateControl> mTransmitRateControl;

    double const W=1023;
    double const AIFS=58*pow(10,-6);
    double const Th=40*pow(10,-6);
    double const EIFS=188*pow(10,-6);
    double const sigma=13*pow(10,-6);

};

} //namespace

#endif
