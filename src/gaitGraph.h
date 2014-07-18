#ifndef GAITGRAPH_H
#define GAITGRAPH_H

#include <iostream>
#include <cmath>

class gaitGraph
{
private:
	//Private Objects
    

	//Private Methods
    
public:
    //Constructor
    gaitGraph();
    
    //Public Methods
    bool isInSwing(int time, int swingStart, int swingEnd);
};

#endif // GAITGRAPH_H
