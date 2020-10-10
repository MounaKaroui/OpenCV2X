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

package artery.application.GLOSA.MultiSegmentCase;

//
// TODO auto-generated module
//
import artery.application.ItsG5Service;
simple MultiSegmentStrategy like ItsG5Service 
{
    //@signal[sent](type="long");  
   // @statistic[sentMsg](title="sentMsg"; source="sent"; record=vector,stats,count; interpolationmode=none);
     
    @signal[receivedCAM](type="long");
    @statistic[receivedCAM](title="receivedCAMs"; source="receivedCAM"; record=vector,stats,count; interpolationmode=none);


    //@signal[rcvGlosa](type="long");
    //@statistic[rcvGlosa](title="rcvPackets"; source="rcvGlosa"; record=vector,stats,count; interpolationmode=none);
    
    int segmentNumber=default(2);
}
