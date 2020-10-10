// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.


#include "CbrRateControl.h"
#include "artery/inet/InetRadioDriver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211ScalarTransmitter.h"
#include "artery/application/Middleware.h"
#include "artery/application/VehicleMiddleware.h"
#include <artery/rateControl/Application/CbrAppTest.h>

#include "inet/physicallayer/common/packetlevel/RadioMedium.h"
#include "inet/linklayer/ieee80211/mac/rateselection/QoSRateSelection.h"

#include "artery/inet/VanetRx.h"
#include "artery/inet/InetMobility.h"
#include "artery/traci/MobilityBase.h"

#include "artery/networking/Router.h"
#include "artery/networking/AdaptiveLimeric/AdaptiveLimericDccEntity.h"

namespace artery {

Define_Module(CbrRateControl);


simsignal_t CbrRateControl::datarateSignal = cComponent::registerSignal("datarate");
simsignal_t  CbrRateControl::cbrSignal=cComponent::registerSignal("cbr");

simsignal_t  CbrRateControl::cbrThSignal_3=cComponent::registerSignal("cbrTh3");
simsignal_t  CbrRateControl::cbrThSignal4_5=cComponent::registerSignal("cbrTh4_5");
simsignal_t  CbrRateControl::cbrThSignal_6=cComponent::registerSignal("cbrTh6");
simsignal_t  CbrRateControl::cbrThSignal_9=cComponent::registerSignal("cbrTh9");
simsignal_t  CbrRateControl::cbrThSignal_12=cComponent::registerSignal("cbrTh12");
simsignal_t  CbrRateControl::cbrThSignal_18=cComponent::registerSignal("cbrTh18");



CbrRateControl::~CbrRateControl()
{
cancelAndDelete(rcSelfMsg);
}


CbrRateControl::CbrRateControl()
{

}


void CbrRateControl::initialize(int stage)
{

    if (stage == INITSTAGE_LOCAL)
    {
    	targetPDR=par("targetPDR").doubleValue();
    	rcSelfMsg=new cMessage("rateControlSelfMessage");
    	interface=par("whichInterface").intValue();
    	//
    	packetSize=par("packetSize").intValue();
    	cModule* host=inet::getContainingNode(this);
    	std::string module=host->getFullName();
    	std::string interfaceName=module+".radioDriver["+std::to_string(interface)+"]";

    	cModule* mod=host->getModuleByPath(interfaceName.c_str());
    	RadioDriverBase* radioDriver=dynamic_cast<RadioDriverBase*>(mod);
    	radioDriver->subscribe(RadioDriverBase::ChannelLoadSignal, this);

        limericIsActive=par("withAdaptiveLimeric").boolValue();
        hysFactor=par("hystersisFactor").doubleValue();
    }
    else if (stage == INITSTAGE_LINK_LAYER_2)
    {

    }
}




const IIeee80211Mode* CbrRateControl::increaseRateIfPossible(const IIeee80211Mode* currentMode)
{
    const IIeee80211Mode *newMode = modeSet->getFasterMode(currentMode);
    return newMode == nullptr ? currentMode : newMode;
}


const IIeee80211Mode* CbrRateControl::decreaseRateIfPossible(const IIeee80211Mode* currentMode)
{
    const IIeee80211Mode *newMode = modeSet->getSlowerMode(currentMode);
    return newMode == nullptr ? currentMode : newMode;
}


void CbrRateControl::getCurrentBitrate()
{
	    cModule* host=inet::getContainingNode(this);
		std::string module=host->getFullName();
		std::string interfaceName=".wlan["+std::to_string(interface)+"]";
		cModule* mod=host->getModuleByPath(interfaceName.c_str());
		bps bitrate=bps{mod->par("bitrate")};
		modeSet = Ieee80211ModeSet::getModeSet(mod->par("opMode").stringValue());
		currentMode= modeSet->findMode(bitrate);
}

void CbrRateControl::emitDatarateSignal()
{
     bps rate = currentMode->getDataMode()->getNetBitrate();
     emit(datarateSignal, rate.get()/1000000);
}




void CbrRateControl::updateBitrate()
{
	cModule* host=inet::getContainingNode(this);
	std::string module=host->getFullName();

	// name of radio driver
	std::string transPath=module+".wlan["+std::to_string(interface)+"].mac.hcf.rateSelection";
	//

	cModule* transmitter=host->getModuleByPath(transPath.c_str());

	QoSRateSelection* qosRateSelection=dynamic_cast<QoSRateSelection*>(transmitter);
	qosRateSelection->dataFrameMode=currentMode;
	qosRateSelection->mgmtFrameMode=currentMode;
	qosRateSelection->multicastFrameMode=currentMode;

}


void CbrRateControl::getNeighboring()
{
    cModule* host=inet::getContainingNode(this);
    std::string module=host->getFullName();
    std::string routerName=".vanetza["+std::to_string(interface)+"]"+".router";
    cModule* mod=host->getModuleByPath(routerName.c_str());
    Router* router=dynamic_cast<Router*>(mod);
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






double CbrRateControl::calculateSucessTransmission(double tau)
{
    return nodeInComm*tau*pow(1-tau,nodeInComm-1);
}

double CbrRateControl::calculateIdleProbability(double tau)
{
    return pow((1-tau),nodeInComm);
}


double CbrRateControl::calculateTargetCBR2(double bitrate)
{
     double targetCbr=0;
     cModule* host=inet::getContainingNode(this);
     std::string module=host->getFullName();
     cModule* midl=host->getModuleByPath((module+".middleware").c_str());
     // Generate  10 packet per sec
     double lamda=1/midl->par("updateInterval").doubleValue();

     getNeighboring();

     double Tc=EIFS+Th+(packetSize*8/bitrate);
     double Ts=AIFS+Th+(packetSize*8/bitrate);
     double tau=2/(W+1);


     double pi=calculateIdleProbability(tau);
     double ps=calculateSucessTransmission(tau);


      double u=lamda*Ts + pow(lamda*Ts,2)/2*(1-lamda*Ts);
      if(nodeInComm>0)
      {
          targetCbr=u + ((1-targetPDR)*Tc)/(pi*sigma+ps*Ts+(1-targetPDR)*Tc);
      }
      return targetCbr;
}

double CbrRateControl::calculateTargetCBR(double bitrate)
{
	cModule* host=inet::getContainingNode(this);
	std::string module=host->getFullName();
	cModule* midl=host->getModuleByPath((module+".middleware").c_str());
	// Generate  10 packet per sec
	double lamda=midl->par("updateInterval").doubleValue()*100;

	getNeighboring();
	//emit(coveredNodes, nodeInComm);
	// n is the number of vehicles in transmission range

	if(nodeInComm>0)
	{
	return (0.5/bitrate)*(nodeInComm-1)*lamda*packetSize*8*(1+targetPDR);
	}

}

void CbrRateControl::getCbrTarget()
{
    if(limericIsActive)
    {
        cModule* host=inet::getContainingNode(this);
        std::string module=host->getFullName();
        std::string dccName=module+".vanetza["+std::to_string(interface)+ "]"+ ".dcc";
        ///
        cModule* dccModule=host->getModuleByPath(dccName.c_str());
        AdaptiveLimericDccEntity* dcc = dynamic_cast<AdaptiveLimericDccEntity*>(dccModule);
        /// lamda is variable (sending message rate)
        cbrTh6=dcc->updateTargetCbr2(6*pow(10,6));
        cbrTh9=dcc->updateTargetCbr2(9*pow(10,6));
        cbrTh12=dcc->updateTargetCbr2(12*pow(10,6));
        cbrTh18=dcc->updateTargetCbr2(18*pow(10,6));
        cbrTh24=dcc->updateTargetCbr2(24*pow(10,6));
    }
    else
    {
        cbrTh6=calculateTargetCBR2(6*pow(10,6));
        cbrTh9=calculateTargetCBR2(9*pow(10,6));
        cbrTh12=calculateTargetCBR2(12*pow(10,6));
        cbrTh18=calculateTargetCBR2(18*pow(10,6));
        cbrTh24=calculateTargetCBR2(24*pow(10,6));
    }

        hys6=calculateHystersis(cbrTh6,cbrTh9);
        hys9 =calculateHystersis(cbrTh9,cbrTh12);
        hys12=calculateHystersis(cbrTh12,cbrTh18);
        hys18=calculateHystersis(cbrTh18,cbrTh24);
}



const IIeee80211Mode*CbrRateControl::increaseToFastestMandMode(const IIeee80211Mode* currentMode)
{
	const IIeee80211Mode* newMode=modeSet->getFastestMandatoryMode();
	return newMode == nullptr ? currentMode : newMode;
}


const IIeee80211Mode* CbrRateControl::inceaseToFastestMode(const IIeee80211Mode*)
{

	const IIeee80211Mode* newMode=increaseToFastestMandMode(currentMode); // to 12 Mbps
	newMode=increaseRateIfPossible(currentMode) ; // to 18 Mbps
	newMode=increaseRateIfPossible(currentMode); // to 24 Mbps
	return newMode==nullptr?currentMode:newMode;

}



double CbrRateControl::calculateHystersis(double v1, double v2)
{

	return (v1-v2)/hysFactor ;

}





void CbrRateControl::adaptBitrate(double cbr)
{

	getCurrentBitrate();
	getCbrTarget();
	double currentBitrate=(currentMode->_getDataMode()->getNetBitrate()).get();

	if(cbr!=0)
	{
		//(currentBitrate==6)&&
	if((currentBitrate==6)&&(cbr<cbrTh6))
	{
	     bitrate=bps{6*pow(10,6)};
	     currentMode=modeSet->findMode(bitrate);
	     updateBitrate();
	     times6_mbps++;
	     recordScalar("ratio6", times6_mbps);

	}
	else if((cbr<cbrTh9)||(cbr>=cbrTh6+hys6))
	{
		bitrate=bps{9*pow(10,6)};
		currentMode=modeSet->findMode(bitrate);
		updateBitrate();
		times9_mbps++;
        recordScalar("ratio9", times9_mbps);

	}
	else if((cbr<cbrTh12)||(cbr>=cbrTh9+hys9))
	{
		bitrate=bps{12*pow(10,6)};
		currentMode=modeSet->findMode(bitrate);
		updateBitrate();
		times12_mbps++;
	    recordScalar("ratio12", times12_mbps);


	}

	else if((cbr<cbrTh18) || (cbr>=cbrTh12+hys12))
	{
		bitrate=bps{18*pow(10,6)};
		currentMode=modeSet->findMode(bitrate);
		updateBitrate();
		times18_mbps++;
	    recordScalar("ratio18", times18_mbps);


	}
	else
	{
		bitrate=bps{24*pow(10,6)};
		currentMode=modeSet->findMode(bitrate);
		updateBitrate();
		times24_mbps++;
	    recordScalar("ratio24", times24_mbps);


	}
	}

	cbr=getCbr();
	emit(cbrSignal,cbr*100);

	emitDatarateSignal();

	emit(cbrThSignal_6,cbrTh6*100);
	emit(cbrThSignal_9,cbrTh9*100);
	emit(cbrThSignal_12,cbrTh12*100);
	emit(cbrThSignal_18,cbrTh18*100);

}


void CbrRateControl::receiveSignal(cComponent* comp, simsignal_t signal, double value, cObject*)
{
	if (signal == RadioDriverBase::ChannelLoadSignal)
	{
		 cModule* module=dynamic_cast<cModule*>(comp);
		 std::string moduleName=module->getFullName();
         std::string radioName="radioDriver["+std::to_string(interface)+ "]";
		 if(moduleName==radioName)
		 {
			  ASSERT(value >= 0.0 && value <= 1.0);
			  vanetza::dcc::ChannelLoad cl{value};
			  adaptBitrate(value);
		 }
	}
}

double CbrRateControl::getCbr()
{
       cModule* host=inet::getContainingNode(this);
       std::string module=host->getFullName();
       // name of radio driver 0 ou 1 it depends on which service we are intressted
       std::string radioName="radioDriver["+std::to_string(interface)+ "]";
       std::string radioDriver=module+"."+radioName;
       ///
       cModule* radMod=host->getModuleByPath(radioDriver.c_str());

       InetRadioDriver* radDriver=dynamic_cast<InetRadioDriver*>(radMod);
       double cbr=(radDriver->cbr);
       return cbr;
}

} //namespace




