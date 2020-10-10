//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See thec
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application.AlertApp;

//
// TODO auto-generated module
//
import artery.application.ItsG5Service;
simple SimpleAlertApp like ItsG5Service
{
    parameters:
        
       double size=default(100); 
       string senderVehId=default("f_0_0.0");
       
       // signals
       @signal[rcvdPk](type=cPacket);   
       @signal[sentPk](type=cPacket);
       
       // statistics
       @statistic[throughput](source="throughput(rcvdPk)/1000000"; record=vector,mean);
       @statistic[rcvdPk](source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       @statistic[sentPk](source=sentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       @statistic[delayAlert](source="messageAge(rcvdPk)"; unit=s; record=stats,vector,mean; interpolationmode=none);
       
    
}
