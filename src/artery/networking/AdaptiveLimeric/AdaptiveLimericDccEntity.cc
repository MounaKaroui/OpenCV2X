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

#include "AdaptiveLimericDccEntity.h"
#include "artery/utility/PointerCheck.h"
#include "artery/rateControl/CbrRateControl.h"

#include "inet/linklayer/ieee80211/mac/rateselection/QoSRateSelection.h"
#include "artery/application/Middleware.h"

#include <artery/rateControl/Application/CbrAppTest.h>
#include "artery/inet/InetRadioDriver.h"
#include "inet/common/ModuleAccess.h"
#include "artery/networking/Router.h"

#include<chrono>
#include<ratio>

namespace artery {

Define_Module(AdaptiveLimericDccEntity);

simsignal_t AdaptiveLimericDccEntity::msgIntervalSignal = cComponent::registerSignal("msgInterval");




void AdaptiveLimericDccEntity::finish()
{
    // free those objects before runtime vanishes
    mTransmitRateControl.reset();
    mAlgorithm.reset();
    DccEntityBase::finish();
}

vanetza::dcc::TransmitRateThrottle* AdaptiveLimericDccEntity::getTransmitRateThrottle()
{
    return notNullPtr(mTransmitRateControl);
}

vanetza::dcc::TransmitRateControl* AdaptiveLimericDccEntity::getTransmitRateControl()
{
    return notNullPtr(mTransmitRateControl);
}



void AdaptiveLimericDccEntity::getNeighboring()
{

    auto router = inet::getModuleFromPar<Router>(par("routerModule"), this);

    if(router->initialized())
    {
    const vanetza::geonet::LocationTable& lt=router->getLocationTable() ;
    if(lt.has_neighbours())
    {
        auto size= std::distance(lt.neighbours().begin(), lt.neighbours().end());
        nodeInComm=size;
        // determine the size of Location table
    }
    else
    {
        nodeInComm=0;
    }
    }
}

double AdaptiveLimericDccEntity::calculateSucessTransmission(double tau)
{
    return nodeInComm*tau*pow(1-tau,nodeInComm-1);
}

double AdaptiveLimericDccEntity::calculateIdleProbability(double tau)
{
    return pow((1-tau),nodeInComm);
}


double AdaptiveLimericDccEntity::updateTargetCbr2(double bitrate)
{
    double targetCbr=0;
    int packetSize=par("packetSize").intValue()*8;
    getNeighboring();

    double Tc=EIFS +Th+(packetSize/bitrate);
    double Ts=AIFS +Th+(packetSize/bitrate);
    double tau=2/(W+1);

    double targetPDR=par("targetPDR").doubleValue();
    double pi=calculateIdleProbability(tau);
    double ps=calculateSucessTransmission(tau);

    double lamda=poisson(0.1);

    if(mTransmitRateControl!=nullptr)
    {
                std::chrono::seconds sec(1);
                Clock::duration lamda1=mTransmitRateControl.get()->m_budget.interval();
                auto value = std::chrono::duration_cast<std::chrono::milliseconds>(lamda1);
                lamda=1.0/(value.count()/1000);
                emit(msgIntervalSignal,(double)value.count()); // ms
    }//adaptive lamda

    double u=lamda*Ts + pow(lamda*Ts,2)/2*(1-lamda*Ts);

    if(nodeInComm>0)
    {
        targetCbr=u + ((1-targetPDR)*Tc)/(pi*sigma+ps*Ts+(1-targetPDR)*Tc);
        mTargetCbr=UnitInterval {targetCbr};
    }
    return targetCbr;

}


double AdaptiveLimericDccEntity::updateTargetCbr(double bitrate)
{
		cModule* parent=getParentModule();
		cModule* host=inet::getContainingNode(parent);
        auto midl = inet::getModuleFromPar<Middleware>(par("midlModule"),host);
        double lamda=1/midl->par("updateInterval").doubleValue();

        if(mTransmitRateControl!=nullptr)
        {
            std::chrono::seconds sec(1);
            Clock::duration lamda1=mTransmitRateControl.get()->m_budget.interval();
            auto value = std::chrono::duration_cast<std::chrono::milliseconds>(lamda1);
            emit(msgIntervalSignal,(double)value.count()); // ms
            double v=(1.0/value.count())*1000;
            lamda=v;
        }//adaptive lamda

        int packetSize=par("packetSize").intValue()*8; // en bits
        double targetPDR=par("targetPDR").doubleValue();
        getNeighboring();

        // update bitrate
        if(nodeInComm>0)
        {
            double val = (0.5/bitrate)*(nodeInComm-1)*lamda*packetSize*(1+targetPDR);
            mTargetCbr=UnitInterval {(0.5/bitrate)*(nodeInComm-1)*lamda*packetSize*(1+targetPDR)};
            return val;
        }
}



void AdaptiveLimericDccEntity::calculateTargetCbr()
{

    if(withDDC)
    {
        cModule* parent=getParentModule();
        cModule* host=inet::getContainingNode(parent);
        auto cbrMod = inet::getModuleFromPar<CbrRateControl>(par("cbrModule"),host);
        cbrMod->getCurrentBitrate();
        double currentBitrate=(cbrMod->currentMode->getDataMode()->getNetBitrate()).get();
        double v=updateTargetCbr2(currentBitrate);

    }else
    {
        double currentBitrate=6*pow(10,6);
        double v=updateTargetCbr2(currentBitrate);
    }
}

void AdaptiveLimericDccEntity::initializeTransmitRateControl()
{
    ASSERT(mRuntime);
    using namespace vanetza::dcc;
    Limeric::Parameters params;
    withDDC=par("withDDC").boolValue();
    calculateTargetCbr();
    params.cbr_target = mTargetCbr;

    mAlgorithm.reset(new Limeric(*mRuntime, params));
    mTransmitRateControl.reset(new LimericTransmitRateControl(*mRuntime, *mAlgorithm));

    mAlgorithm->on_duty_cycle_change = [this](const Limeric*, vanetza::Clock::time_point)
    {
        mTransmitRateControl->update();
    };
}

void AdaptiveLimericDccEntity::onGlobalCbr(vanetza::dcc::ChannelLoad cbr)
{
    ASSERT(mAlgorithm);
    mAlgorithm->update_cbr(cbr);
}


} //namespace
