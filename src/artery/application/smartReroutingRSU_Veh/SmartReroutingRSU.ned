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

package artery.application.smartReroutingRSU_Veh;

//
// TODO auto-generated module
//
import artery.application.ItsG5Service;
simple SmartReroutingRSU like ItsG5Service
{
    
    parameters:
       int messageSize=default(300);	    
       double slowDownSpeed=default(5.5);
       string damagedVehId=default("f_0_0.0");
       // route Id of an accident zone
       string accidentZone=default("-10114#1 -10114#0 10107 10053#2 10053#0 -10071#5 10065#0 10065#1 10065#2 10102 10032 10072 -10079");
       
       
       @signal[sentPk](type=cPacket);
       @statistic[sentPk](source=sentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       
       //@signal[mode4SentPk](type=cPacket);
       //@statistic[mode4SentPk](source=mode4SentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       
       
       
       
        
    
}
