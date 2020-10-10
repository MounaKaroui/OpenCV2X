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

simple GlosaUseCaseRSU like ItsG5Service
{
    
    
    parameters:
    int size=default(500);
    double tlsDist1=default(448.78013149889);
    double tlsDist2=default(868.56994710885);
    double tlsDist3=default(1441.229);
    double tlsDist4=default(2010.119195734);
    double tlsDist5=default(2419.8853290552);
    
    
    string vanetzaModule=default("^.vanetza");   
     
    @signal[sentPk](type=cPacket);
	@statistic[sentPk](title="packets sent"; source=sentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
       
	
}
