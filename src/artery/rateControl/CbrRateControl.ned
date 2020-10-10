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

package artery.rateControl;

//
// TODO auto-generated module
//

import inet.linklayer.ieee80211.mac.contract.IRateControl;
simple CbrRateControl
{
    
      parameters:
        @display("i=block/cogwheel");

		//
		int whichInterface=default(1); 
        double hystersisFactor=default(10);
        
        //Target PDR
        double targetPDR=default(0.99); 
        string radioMed=default("^.radioMedium");
        string mobilityModule=default("^.mobility");       
        bool withAdaptiveLimeric=default(false);
        int packetSize=default(500);
        // statistics 
        @signal[datarate];
        @statistic[datarate](record=vector;source="datarate");
       
        @signal[cbr](type=double);
        @statistic[cbrVec](title="cbr";record=vector;source="cbr");
     	
     	@signal[cbrTh3];
     	@statistic[cbrTh_3](title="cbrTh3";record=vector;source="cbrTh3");
     	
     	@signal[cbrTh4_5];
     	@statistic[cbrTh4_5](title="cbrTh4_5";record=vector;source="cbrTh4_5");
     	
     	@signal[cbrTh6];
     	@statistic[cbrTh_6](title="cbrTh6";record=vector,mean;source="cbrTh6");
     	
     	@signal[cbrTh9];
     	@statistic[cbrTh_9](title="cbrTh9";record=vector,mean;source="cbrTh9");
     	
     	@signal[cbrTh12];
     	@statistic[cbrTh_12](title="cbrTh12";record=vector,mean;source="cbrTh12");
     	
     	@signal[cbrTh18];
     	@statistic[cbrTh_18](title="cbrTh18";record=vector,mean;source="cbrTh18");
     	
     	
     	
     	
     	
     	
     	
}
