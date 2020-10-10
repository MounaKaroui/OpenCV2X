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

#include "SimpleSegmentSABIN.h"



namespace artery {


using namespace omnetpp;
using namespace vanetza;
namespace si = boost::units::si;

Define_Module(SimpleSegmentSABIN);

/////
simsignal_t SimpleSegmentSABIN::rcvdPkSignal = registerSignal("rcvdPk");
////
simsignal_t   SimpleSegmentSABIN::wtSignal=cComponent::registerSignal("waitingTime");
simsignal_t   SimpleSegmentSABIN::fuelSignal=cComponent::registerSignal("fuel");


void SimpleSegmentSABIN::initialize()
{

    ItsG5Service::initialize();

    xlimit1=par("xlimit1").doubleValue();
    xlimit2=par("xlimit2").doubleValue();
    xlimit3=par("xlimit3").doubleValue();
    xlimit4=par("xlimit4").doubleValue();
    xlimit5=par("xlimit5").doubleValue();


}




// To check the distance until the corresponding traffic light
double SimpleSegmentSABIN::getTravelDistance(GlosaPacket* glosaPacket)
{
    // example until 5 consecutives traffic lights
    double travelDistance=vehicle->getLiteAPI().vehicle().getDistance(vehicle->getVehicleId());
    double xveh=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId()).x;

    if((glosaPacket->getSenderId()==0)&&(xveh>xlimit1))
    {
        return (abs(glosaPacket->getDistancetl1()-travelDistance));

    }


    if((glosaPacket->getSenderId()==1)&&((xveh<xlimit1)&&(xveh>xlimit2)))
    {
        return (abs(glosaPacket->getDistancetl2()-travelDistance));
    }


    if((glosaPacket->getSenderId()==2)&&((xveh<xlimit2)&&(xveh>xlimit3)))
    {
        return (abs(glosaPacket->getDistancetl3()-travelDistance));
    }


    if((glosaPacket->getSenderId()==3)&&((xveh<xlimit3)&&(xveh>xlimit4)))
    {
        return (abs(glosaPacket->getDistancetl4()-travelDistance));
    }

    if((glosaPacket->getSenderId()==4)&&((xveh<xlimit4)&&(xveh>xlimit5)))
    {
        return (abs(glosaPacket->getDistancetl5()-travelDistance));
    }
}

double SimpleSegmentSABIN::speedDecision(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT)
{

    double t1=glosaPacket->getStartGreenTime();
    double t2=glosaPacket->getEndGreenTime();
    double travelDistance=getTravelDistance(glosaPacket);
    double advisorySpeed=0;
    glosa->calculateSpeedBorder(t1,t2,t0,travelDistance,v0,deltaT);

    double n=0;
    while((glosa->boundarySpeeds.v1==-1)&&(n<3))
    {
        n++;
        glosa->calculateSpeedBorder(t1+n*glosa->cycleDuration, t2+n*glosa->cycleDuration,t0,travelDistance, v0,deltaT);
    }


    if((glosa->boundarySpeeds.v1==-1)||(glosa->boundarySpeeds.v1==0))
    {
        advisorySpeed=glosa->minSpeed;
    }

    if((glosa->boundarySpeeds.v1!=-1)&&(glosa->boundarySpeeds.v1!=0))
    {
        advisorySpeed=glosa->boundarySpeeds.v1;
    }

    return advisorySpeed;

}

double SimpleSegmentSABIN::verifSpeed(double speed, Glosa* glosa)
{
    if(speed<glosa->minSpeed)
    {
        return glosa->minSpeed;
    }
    else
    {
        return speed;
    }

}


void SimpleSegmentSABIN::speedDecider(GlosaPacket* glosaPacket, Glosa* glosa, double t0, double v0, double deltaT)
{
    std::vector<libsumo::TraCINextTLSData> vehicleTLSData=vehicle->getLiteAPI().vehicle().getNextTLS(vehicle->getVehicleId());
    int size=vehicleTLSData.size();
    double v1s1, v1s2, v1s3,v1s4,v1s5;


    if(size==0)
    {
        vehicle->setSpeed(vehicle->getMaxSpeed());
        arrivalTimeS3=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;

    }
    else
    {
        double xveh=vehicle->getLiteAPI().vehicle().getPosition(vehicle->getVehicleId()).x;

        if((glosaPacket->getSenderId()==0)&&(xveh>xlimit1)){

            v1s1=speedDecision(glosaPacket,glosa, t0, v0, deltaT);

            if(v1s1!=0)
            {
                v1s1=verifSpeed(v1s1, glosa);
                vehicle->setSpeed(v1s1*si::meter_per_second);
            }
        }
        if((glosaPacket->getSenderId()==1)&&((xveh>xlimit2)&&(xveh<=xlimit1)))
        {

            v1s2=speedDecision(glosaPacket,glosa, t0, v0, deltaT);

            if(v1s2!=0)
            {
                v1s2=verifSpeed(v1s2, glosa);
                vehicle->setSpeed(v1s2*si::meter_per_second);
                arrivalTimeS1=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;

            }
        }
        if((glosaPacket->getSenderId()==2)&&((xveh>xlimit3)&&(xveh<xlimit2)))
        {
            v1s3=speedDecision(glosaPacket,glosa, t0, v0, deltaT);
            if(v1s3!=0)
            {
                v1s3=verifSpeed(v1s3, glosa);
                vehicle->setSpeed(v1s3*si::meter_per_second);
                arrivalTimeS2=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
            }
        }

        if((glosaPacket->getSenderId()==3)&&((xveh>xlimit4)&&(xveh<xlimit3)))
        {

            v1s4=speedDecision(glosaPacket,glosa, t0, v0, deltaT);
            if(v1s4!=0)
            {

                v1s4=verifSpeed(v1s4, glosa);
                vehicle->setSpeed(v1s4*si::meter_per_second);
                arrivalTimeS4=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;

            }
        }

        if((glosaPacket->getSenderId()==4)&&((xveh>xlimit5)&&(xveh<xlimit4)))
        {

            v1s5=speedDecision(glosaPacket,glosa, t0, v0, deltaT);
            if(v1s5!=0)
            {
                v1s5=verifSpeed(v1s5, glosa);
                vehicle->setSpeed(v1s5*si::meter_per_second);
                arrivalTimeS5=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
            }
        }
    }


}






void SimpleSegmentSABIN::indicate(const vanetza::btp::DataIndication&,  cPacket* packet)
{
    double v0=0;
    double t0=0;
    Glosa* glosa=new Glosa();
    vehicle=getFacilities().get_mutable_ptr<traci::VehicleController>();
    glosa->accel=vehicle->getLiteAPI().vehicle().getAccel(vehicle->getVehicleId());
    glosa->decel=(-1)*vehicle->getLiteAPI().vehicle().getDecel(vehicle->getVehicleId());
    glosa->maxSpeed=vehicle->getLiteAPI().vehicle().getMaxSpeed(vehicle->getVehicleId());
    glosa->minSpeed=5.5;
    glosa->cycleDuration=60;
    glosa->vehicle=vehicle;
    v0=vehicle->getSpeed()/si::meter_per_second;

    t0=vehicle->getLiteAPI().simulation().getCurrentTime()*0.001;
    double deltaT=5.5;
    GlosaPacket* glosaPacket = dynamic_cast<GlosaPacket*>(packet);

    if((glosaPacket->getFlag()==1))
    {
        emit(rcvdPkSignal,glosaPacket);
        speedDecider(glosaPacket, glosa, t0, v0, deltaT);
    }

    calculateStats();
}


void SimpleSegmentSABIN::calculateStats()
{
    double speed=vehicle->getSpeed()/si::meter_per_second;
    double fc=vehicle->getLiteAPI().vehicle().getFuelConsumption(vehicle->getVehicleId());
    double wt=vehicle->getLiteAPI().vehicle().getWaitingTime(vehicle->getVehicleId());
    emit(fuelSignal,fc);
    emit(wtSignal,wt);
    totalSpeed.record(speed);
}


void SimpleSegmentSABIN::finish()
{
    ItsG5Service::finish();
}




}
