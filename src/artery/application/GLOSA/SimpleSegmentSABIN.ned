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

package artery.application.GLOSA;

//
// TODO auto-generated module
//
import artery.application.ItsG5Service;
simple SimpleSegmentSABIN  like ItsG5Service
{
  
    parameters:
      
    double xlimit1=default(2465);
    double xlimit2=default(2066); 
    double xlimit3=default(1493);
    double xlimit4=default(932);
    double xlimit5=default(532);
        
 	@class(SimpleSegmentSABIN);  
 
 	
    @signal[rcvdPk](type=cPacket);        
    @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
     
        
    @statistic[throughput](title="throughput"; unit=bps; source="throughput(rcvdPk)/1000000"; record=vector,mean);
    @statistic[delay](title="received packet delay"; source="messageAge(rcvdPk)"; unit=s; record=stats,vector,mean; interpolationmode=none);

    @signal[fuel];
    @statistic[fuel](source="fuel";record=vector,mean);
    @signal[waitingTime];    
    @statistic[waitingTime](source="waitingTime";record=vector,mean);
 
}
