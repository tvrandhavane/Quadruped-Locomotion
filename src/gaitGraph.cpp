#include "gaitGraph.h"

gaitGraph::gaitGraph(){
	
}

bool gaitGraph::isInSwing(int time, int swingStart, int swingEnd){
	if(swingStart < swingEnd){
		if(time >= swingStart && time < swingEnd){
			return true;
		}
	}
	else if(swingStart == swingEnd){
		return true;
	}
	else{
		if(time >= swingStart){
			return true;
		}
		else if(time < swingEnd){
			return true;
		}
	}
	return false;
}