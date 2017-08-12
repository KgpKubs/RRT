#include <iostream>
#include "./../include/RRTConnect.hpp"

using namespace std;
using namespace rrt;

bool check(Utils::Point<int> cur)
{
	if(abs(cur.x)<2000 && abs(cur.y)<2000){
		// if (cur.x < 550 && cur.x > 450 && cur.y < 550 & cur.y > 450)
		// {
		// 	return false;
		// }
		return true;
	}
	return false;
}

int main()
{
	srand(time(NULL));
	Utils::Point<int> start,finish,origin;
	start.x=30;
	start.y=30;
	finish.x=1500;
	finish.y=1730;
	origin.x=0;
	origin.y=0;

	RRTConnect<int> test;
	test.setEndPoints(start,finish);
	test.setCheckPointFunction(*(check));
	test.setStepLength(50);
	test.setHalfDimensions(2000.0,2000.0);
	test.setBiasParameter(100);
	test.setOrigin(origin);
	test.setMaxIterations(10000);
	test.plan();
	// test1.setEndPoints(start,finish);
	// test1.setCheckPointFunction(*(check));
	// test1.setStepLength(200);
	// test1.setHalfDimensions(2000.0,2000.0);
	// test1.setBiasParameter(100);
	// test1.setOrigin(origin);
	// test1.setMaxIterations(10000);
	// test2.plan();

	vector<Utils::Point<int> > path=test.getPointsOnPath();
	cout<<"#################################################"<<endl;
	for(int i=0;i<path.size();i++)
		cout<<"("<<path[i].x<<","<<path[i].y<<")";

	// cout<<"sfd"<<connectA.x<<","<<connectA.y<<" "<<connectB.x<<","<<connectB.y;
}	