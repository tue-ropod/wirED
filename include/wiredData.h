#include "ed/arbitraryDataBuffer.h"
#include "wire_msgs/WorldEvidence.h"

#include <stdio.h>

/*
// EXAMPLE
template <typename T>
class Foo : public Bar
{
T member_;
public:
   Foo(){}
   T& member() {return member_;}
   virtual bar() {printf("bar");}
};
*/

//template <typename T>
class wiredData : public ArbitrayDataBuffer
{
//wire_msgs::WorldEvidence data;

int bufferSize; // TODO make configurable?

bounded_buffer<wire_msgs::WorldEvidence> data_buf; // TODO make pointer? Make buffer size variable

public:
   wiredData(  ): bufferSize(100), data_buf(bufferSize) {}        // verplaatsen naar base class?
   
   bounded_buffer<wire_msgs::WorldEvidence>& getDataDerived() {return data_buf;}
  // virtual bar() {printf("bar");}
   std::string getName() {return name;}
   
    std::string getType(){ return "wiredData"; };
};