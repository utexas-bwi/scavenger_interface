#include "hunter.h"

class PassiveHunter : public Hunter {
public:
	PassiveHunter();	
	void performHunt() const;
	void performTask() const;
	void performFindTask(){}
	void performLocationTask(){}
	void performTravelTask(){}
};  
