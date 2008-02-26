#include "SimpleMacLayer.h"
Define_Module(SimpleMacLayer);

//---omnetpp part----------------------

//---intialisation---------------------
void SimpleMacLayer::initialize(int stage) {
	BaseModule::initialize(stage);
	
	if(stage == 0) {
		hostCount = simulation.systemModule()->par("numHosts");
		myIndex = findHost()->index();
		
		dataOut = findGate("lowerGateOut");
		dataIn = findGate("lowerGateIn");
		
		phy = FindModule<MacToPhyInterface*>::findSubModule(this->parentModule());
		
	} else if(stage == 1) {
		if(myIndex == 0){
			log("Switching radio to TX...");
			phy->setRadioState(Radio::TX);
		}else{
			log("Switching radio to RX...");
			phy->setRadioState(Radio::RX);
		}
	}
}

void SimpleMacLayer::handleMessage(cMessage* msg) {
	
	if(msg->kind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
		log("...switching radio done.");
		switch(phy->getRadioState()) {
		case Radio::TX:
			broadCastPacket();
			break;
		default:
			break;
		}
		delete msg;
		
	} else if (msg->kind() == TEST_MACPKT) {
		handleMacPkt(static_cast<MacPkt*>(msg));
	} else if(msg->kind() == MacToPhyInterface::TX_OVER) {
		handleTXOver();
		delete msg;
	}
	
}

void SimpleMacLayer::handleTXOver() {
	log("Transmission over signal from PhyLayer received. Changing back to RX");
	phy->setRadioState(Radio::RX);
}

void SimpleMacLayer::handleMacPkt(MacPkt* pkt) {
	//TODO: do things
	if(pkt->getDestAddr() == myIndex){
		log("Received MacPkt for me - broadcasting answer (but first change to TX mode)");	
		phy->setRadioState(Radio::TX);
	}else
		log("Received MacPkt - but not for me.");
	
	delete pkt;
	
}

void SimpleMacLayer::log(std::string msg) {
	ev << "[Host " << myIndex << "] - MacLayer: " << msg << endl;
}

void SimpleMacLayer::broadCastPacket() {
	MacPkt* pkt = createMacPkt(3.0);
	
	log("Broadcast packet with duration 3.0 send to phy layer.");
	sendDown(pkt);
}

void SimpleMacLayer::sendDown(MacPkt* pkt) {
	send(pkt, dataOut);
}

MacPkt* SimpleMacLayer::createMacPkt(simtime_t length) {
	Signal* s = new Signal(simTime(), length);
	MacToPhyControlInfo* ctrl = new MacToPhyControlInfo(s);
	MacPkt* res = new MacPkt();
	res->setControlInfo(ctrl);
	res->setKind(TEST_MACPKT);
	res->setDestAddr((myIndex + 1) );
	return res;
}
