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

packet GlosaPacket {
	
	string tlsId;
	int senderId; 
	string receiverId;
	string state;
	int distancetl1;
	int distancetl2;
	int distancetl3;
	int distancetl4;
	int distancetl5;
	int distancetl6;
	int flag;
	int size;
	int tlsIndex;
	
	double x;
	double y; 
	double z;
	
	double startGreenTime;
	double endGreenTime;
}