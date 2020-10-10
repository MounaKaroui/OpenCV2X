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

#ifndef __ARTERY_ARTERY_IDE_GLOSA_H_
#define __ARTERY_ARTERY_IDE_GLOSA_H_

#include "artery/application/ItsG5Service.h"
#include <vanetza/units/velocity.hpp>
#include "traci/LiteAPI.h"
#include <vanetza/asn1/cam.hpp>
#include <stdlib.h>
#include "artery/utility/GlosaPacket_m.h"
#include<omnetpp.h>

#include "artery/traci/VehicleController.h"


using namespace omnetpp;
using namespace vanetza;

/**
 * TODO - Generated class
 */
namespace artery {
class Glosa: public  ItsG5Service
{
    public:

    struct boundarySpeed
    {
        double v1, v2;
    };

	struct vectorStats{

           cOutVector travelTimeVec;
           cOutVector  departTime;
           cOutVector vehPosition;
           cOutVector cSpeed;
           cOutVector speedAvg;
           cOutVector speedVerif;

           cOutVector fuelC;
           cOutVector waitingTimeVec;
           cOutVector startGreenTimeVec;
           cOutVector endGreenTimeVec;

           cOutVector v1_vec;
           cOutVector v2_vec;
           cOutVector emVec;

           cOutVector tpmax_vec;
           cOutVector tpmin_vec;


           cOutVector stoppedVehVector;

           cOutVector stoppedVehiclePositionX;
           cOutVector stoppedVehiclePositionY;
           cOutVector stoppedVehiclePositionZ;
	};

	cOutVector xPos;
	cOutVector distanceToTls;
    vectorStats stats;
	simsignal_t  receivedSignal;
	double T1;

    long receivedPacketFromRSU;
    boundarySpeed boundarySpeeds;

    std::string theMostAdequateTLSId;
    int tlsIndex;
    double cycleDuration;
    bool s1,s2,s3;

    double accel;
    double decel;
    double minSpeed;
    double maxSpeed;
    double distanceToTrafficLight;


    double endGreenTime;
    double startGreenTime;

    cOutVector senderId;
    long sentPacketFromVeh;
    long stoppedVehTime;






    	void rerouteByQueue(std::string tlsId);
        Glosa();
        void  virtual indicate(const vanetza::btp::DataIndication&,  cPacket* packet) override;
        void virtual trigger() override;
        void  virtual receiveSignal(cComponent*, omnetpp::simsignal_t, cObject* obj, cObject*) override;
        double calculateMinArrivalTime(double t0, double v0,double d);

        double calculateSpeed(double t,double d,double v0);
        void  calculateSpeedBorder(double t1,double t2,double t0,double d,double v0, double deltaT);

        traci::VehicleController* vehicle;


    protected:
        void virtual initialize() override;
        void virtual finish() override;
        void virtual handleMessage(omnetpp::cMessage*) override;


    private:
        omnetpp::cMessage* m_self_msg;


};
}
#endif /* GLOSA_H_ */
