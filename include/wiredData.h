#include "ed/arbitraryDataBuffer.h"
#include "wire_msgs/WorldEvidence.h"

#include <stdio.h>

class wiredDataBuffer : public ArbitrayDataBuffer
{

bounded_buffer<wire_msgs::WorldEvidence> data_buf; //TODO Make buffer size variable?

public:
   wiredDataBuffer( int bufferSizeIn = 100 ): ArbitrayDataBuffer(bufferSizeIn), data_buf(bufferSize) { }      
   
   // Use this function in order to obtain the data. Policy is to wait if data are available. If a fixed rate is desired (if it is not desired to wait untill 
   // new data become available, then check with the .is_not_empty()-method of the bounded_buffer.
   bounded_buffer<wire_msgs::WorldEvidence>& getDataDerived() { return data_buf;}   // TODO? Move to base class by adding a templated function?
};