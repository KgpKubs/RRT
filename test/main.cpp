#include <iostream>
#include "./../include/DRRT.hpp"
using namespace std;
using namespace rrt;

bool check(Utils::Point<int> cur)
{
	if(abs(cur.x)<2000 && abs(cur.y)<2000){
		Utils::Point<int> a,b;
		a.x = 530;
		a.y = 570;
		b = cur;
		double dist = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
		return true;
	}
	cout<<"False check";
	return false;
}

int main()
{
	srand(time(NULL));
	Utils::Point<int> start,finish,origin,checkpt;
	start.x=30;
	start.y=30;
	finish.x=1500;
	finish.y=1730;
	origin.x=0;
	origin.y=0;
	checkpt.x = 400;
	checkpt.y = 0;

	// steplength << bucketsize (finding child using bucket)
	DRRT<int> test;
	test.setEndPoints(start,finish);
	test.setCheckPointFunction(*(check));
	test.setStepLength(50);
	test.setHalfDimensions(2000.0,2000.0);
	test.setBucketSize(100);
	test.setPointsInBucket(3);
	test.setBiasParameter(100);
	test.setOrigin(origin);
	test.setMaxIterations(50000);
	test.generateGrid();
	test.setTimeOut(0.010);
	Utils::Point<int> ob;
	ob.x = 200;
	ob.y = 0;

	
	test.plan();
	vector<Utils::Point<int> > path;

	float starttime = clock();
	path =test.getPath(start,checkpt);
	float endtime = clock();

	cout<<"Time taken = "<<(endtime - starttime)/CLOCKS_PER_SEC;
	cout<<"\n"<<path.size();
	cout<<"###########################IN Main######################"<<endl;
	for(int i=0;i<path.size();i++)
		cout<<"("<<path[i].x<<","<<path[i].y<<")";

	cout<<endl<<endl;

	
}	