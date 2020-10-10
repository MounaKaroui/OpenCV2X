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
simple SmartReroutingVeh like ItsG5Service
{
    
    parameters:
       
       @signal[rcvdPk](type=cPacket);   
       @statistic[throughput](source="throughput(rcvdPk)/1000000"; record=vector,mean);
       @statistic[rcvdPk](source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       @statistic[delay](source="messageAge(rcvdPk)"; unit=s; record=stats,vector,mean; interpolationmode=none);
       
       @signal[routedVeh];
       @statistic[routedVeh](source="routedVeh"; record=vector,mean);
       
        
    
}
