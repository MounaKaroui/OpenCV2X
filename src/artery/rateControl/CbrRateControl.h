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

#ifndef __ARTERY_ARTERY_IDE_CBRRATECONTROL_H_
#define __ARTERY_ARTERY_IDE_CBRRATECONTROL_H_

#include <omnetpp.h>
#include "inet/linklayer/ieee80211/mac/ratecontrol/RateControlBase.h"
#include "artery/nic/ChannelLoadMeasurements.h"

//using namespace omnetpp;
using namespace inet::ieee80211;
using namespace inet;
class RadioDriverBase;

namespace artery {
class CbrRateControl : public omnetpp::cSimpleModule, public omnetpp::cListener
{

protected:



	double targetPDR=0;
	double cbrTh3=0;
	double cbrTh4_5=0;
	double cbrTh6=0;

	double cbrTh9=0;
	double cbrTh12=0;
	double cbrTh18=0;
    double cbrTh24=0;


    double hys6;
    double hys9;
    double hys12;
    double hys18;

    double hysFactor;

	bool limericIsActive;
	int interface;
	int packetSize;
	double calculateHystersis(double v1, double v2);
	void getNeighboring();

	double calculateIdleProbability(double tau);
	double calculateSucessTransmission(double tau);

	void getCbrTarget();

	double controlPeriod;

	//bps bitrate;

	double calculateTargetCBR(double bitrate);
	double calculateTargetCBR2(double bitrate);
	const IIeee80211Mode* inceaseToFastestMode(const IIeee80211Mode*);
	const IIeee80211Mode* increaseToFastestMandMode(const IIeee80211Mode*);
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    const IIeee80211Mode* increaseRateIfPossible(const IIeee80211Mode* currentMode);
    const IIeee80211Mode* decreaseRateIfPossible(const IIeee80211Mode* currentMode);
    void updateBitrate();
    void emitDatarateSignal();
    void adaptBitrate(double cbr);
    void receiveSignal(cComponent*, simsignal_t signal, double value, cObject*) override;

   public:
       CbrRateControl();
       ~CbrRateControl();
       void getCurrentBitrate();
       const IIeee80211Mode *currentMode;
       const Ieee80211ModeSet *modeSet;
       bps bitrate;
       double getCbr();
       double nodeInComm=0;
       static simsignal_t datarateSignal;
       static simsignal_t cbrSignal;



       static simsignal_t cbrThSignal_3;
       static simsignal_t cbrThSignal4_5;
       static simsignal_t cbrThSignal_6;
       static simsignal_t cbrThSignal_9;
       static simsignal_t cbrThSignal_12;
       static simsignal_t cbrThSignal_18;

   private:

       cMessage* rcSelfMsg;
       double const W=1023;
       double const AIFS=58*pow(10,-6);
       double const Th=40*pow(10,-6);
       double const EIFS=188*pow(10,-6);
       double const sigma=13*pow(10,-6);

       double times6_mbps=0;
       double times9_mbps=0;
       double times12_mbps=0;
       double times18_mbps=0;
       double times24_mbps=0;
};

} //namespace

#endif
