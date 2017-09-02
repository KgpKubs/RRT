
#include <bits/stdc++.h>
#include "./RRT.hpp"
#include "./RRT_implementation.hpp"
#include "./utils.hpp"

using namespace std;
namespace rrt
{

	template<class T>
	void DRRT<T>::setBucketSize(unsigned int size){
		BucketSize = size;
	}

	template<class T>
	void DRRT<T>::setPointsInBucket(unsigned int num){
		PointsInBucket = num;
	}

	template <class T>
	void DRRT<T>::generateGrid(){
		numX = 2*halfDimensionX/BucketSize;
		if (numX != (int)numX)
		{
			numX = (int)numX + 1;
		}
		numY = 2*halfDimensionY/BucketSize;
		if (numY != (int)numY)
		{
			numY = (int)numY + 1;
		}
		int num = numX*numY;
		for (int i = 0; i < num; ++i)
		{
			std::vector<int> v;
			bucket.push_back(std::pair<int,std::vector<int> > (i,v));
		}
	}

	template < class T>
	int DRRT<T>::findBucket(Utils::Point<T> pt){
		int x = pt.x + halfDimensionX ;
		int y = pt.y + halfDimensionX ;
		int numx = (int)(x/BucketSize) + 1;
		if (numx > numX)
		{
			numx = numx - 1;
		}
		int numy = (int)(y/BucketSize) + 1;
		if (numy > numY)
		{
			numy = numy - 1;
		}
		int num = (numy - 1)*numX + numx - 1;
		return num;
	}

	template <class T>
	void DRRT<T>::setEndPoints(Utils::Point<T> start, Utils::Point<T> end)
	{
		startPoint=start;
		endPoint=end;
	}

	template <class T>
	void DRRT<T>::setTimeOut(float time)
	{
		timeOut = time;
	}

	template <class T>
	void DRRT<T>::setHalfDimensions(double a, double b)
	{
		halfDimensionX=a;
		halfDimensionY=b;
	}

	template <class T>
	void DRRT<T>::setOrigin(Utils::Point<T> pt)
	{
		origin=pt;
	}

	template <class T>
	std::vector<Utils::Point<T> > DRRT<T>::getPointsOnPath(Utils::Point<T> goal)
	{
		// clear vector before use
		pathPoints.clear();
		Utils::Point <T> cur;
		int numBucket = findBucket(goal);
		float minDist = 9999;
		int min = -1;

		// checking point at minimum distance from goal
		// in goal's bucket, 
		// If any change......  do here
		for (int i = 0; i < bucket[numBucket].second.size(); ++i)
		{
			int id = bucket[numBucket].second[i];
			float Dist = dist(goal,tree[id].first);
			if (Dist < minDist)
			{
				minDist = Dist;
				min = id;
			}
		}
		if (goal != tree[min].first)
		{
			pathPoints.push_back(goal);
		}
		cur = tree[min].first;
		while(cur != getParent(cur)){
			pathPoints.push_back(cur);
			cur = getParent(cur);
		}
		pathPoints.push_back(cur);
		return pathPoints;
	}

	template <class T>
	std::vector<Utils::Point<T> > DRRT<T>::getPath(Utils::Point<T> start,Utils::Point<T> goal){
		reWireRoot(start);
		std::vector<Utils::Point<T> > path;
		cout<<"Current Root"<<root.x<<" "<<root.y<<endl;
		path = getPointsOnPath(goal);
		if (path[path.size() - 1] != start)
		{
			path.push_back(start);
		}
		return path;
	}

	template <class T>
	void DRRT<T>::setStepLength(double length)
	{
		stepLength=length;
	}

	template <class T>
	void DRRT<T>::setMaxIterations(int maxIt)
	{
		maxIterations=maxIt;
	}

	
	template<class T>
	bool DRRT<T>::saturation(){
		for (int i = 0; i < bucket.size(); ++i)
		{
			if (bucket[i].second.size() < PointsInBucket)
			{
				return true;
			}
		}

		return false;
	}

	template<class T>
	bool DRRT<T>::dense(Utils::Point<T> pt){
		int numBucket = findBucket(pt);
		int density = bucket[numBucket].second.size();
		if (density > PointsInBucket)
		{
			return false;
		}
		return true;

	}

	template <class T>
	bool DRRT<T>::collision(Utils::Point<T> pt){
		for (int i = 0; i < obstacles.size(); ++i)
		{
			std::vector<int> obstacleBucket;
			Utils::Point<T> ob = obstacles[i].first;
			int radius = obstacles[i].second;
			int numbucket = findBucket(ob);
			obstacleBucket.push_back(numbucket);
			if (numbucket%(int)numX == 0)
			{
				obstacleBucket.push_back(numbucket + 1);
			}
			else if(numbucket % (int)numX == (int)numX - 1)
				obstacleBucket.push_back(numbucket - 1);
			else{
				obstacleBucket.push_back(numbucket + 1);
				obstacleBucket.push_back(numbucket - 1);
			}
			
			if (numbucket/(int)numX == 0)
			{
				obstacleBucket.push_back(numbucket + (int)numX);	
			}

			else if(numbucket/(int)numX == (int)numY - 1){
				obstacleBucket.push_back(numbucket - (int)numX);
			}

			else{
				obstacleBucket.push_back(numbucket + (int)numX);
				obstacleBucket.push_back(numbucket - (int)numX);
			}

			for (int n = 0; n < obstacleBucket.size(); ++n)
			{
				int numBucket = obstacleBucket[n];
				for (int j = 0; j < bucket[numBucket].second.size(); ++j)
				{
					int id = bucket[numBucket].second[j];
					Utils::Point<T> node = tree[id].first;
					if (dist(pt,node) < radius)
					{
						cout<<"Obstacle\n";
						return false;
					}
				}
			}
		}
		

		return true;
	}

	template <class T>
	bool DRRT<T>:: plan()
	{
		// unordered_map<Utils:Point<T>, float > costMap;
		cout<<"Planning\n";
		int count=0;
		int check=0;
		int numBucket = findBucket(startPoint);
		bucket[numBucket].second.push_back(tree.size());		
		//storing index of node in bucket
		tree.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (startPoint,startPoint));
		root = startPoint;
		Utils::Point<T> next;
		float start_time = clock();
		while( check < maxIterations && saturation())
		{
			do{
				next = generatePoint();
			}while(checkPoint(next)==false || dense(next)==false);
			int numBucket = findBucket(next);
			/*storing index of node in bucket*/
			bucket[numBucket].second.push_back(tree.size());	
			growTree(next);
			check ++;
			
		}
		float end_time = clock();

		
		cout<<tree.size();
		std::cout<<"\nTime to generate path = "<<(float)(end_time-start_time)/CLOCKS_PER_SEC<<std::endl;

		// for (int i = 0; i < tree.size(); ++i)
		// {
		// 	cout<<"("<<tree[i].first.x<<","<<tree[i].first.y<<")";
		// }
		cout<<endl;
		return true;


	}

	template <class T>

	void DRRT<T>::growTree(Utils::Point<T> temp)
	{
		Utils::Point<T> parent;
		parent = findClosestNode(temp);
		tree.push_back(std::pair< Utils::Point<T>,Utils::Point<T> >(temp,parent));
	}
	

	template <class T>
	void DRRT<T>::setCheckPointFunction(bool (*f)(Utils::Point<T>))
	{
		userCheck=f;
	}

	template <class T>
	Utils::Point<T> DRRT<T>::findClosestNode(Utils::Point<T> cur)
	{
		double min_dist=INT_MAX;
		Utils::Point<T> closest;
		for(int i=0;i<tree.size();i++)
		{
			if(dist(tree[i].first,cur) < min_dist )
			{
				min_dist=dist(tree[i].first,cur);
				closest=tree[i].first;
			}
		}
		return closest;
	}

	template <class T>
	Utils::Point<T> DRRT<T>::generatePoint()
	{
		Utils::Point<T> temp;
		int signX,signY;

		if(rand()%2==0)
			signX=-1;
		else
			signX=1;
		if(rand()%2==0)
			signY=-1;
		else 
			signY=1;
		temp.x = origin.x + signX* ( rand() % (int)halfDimensionX );
		temp.y = origin.y + signY* ( rand() % (int)halfDimensionY );


		Utils::Point<T> closest=findClosestNode(temp);
		double theta = atan2(temp.y-closest.y,temp.x-closest.x);
		double dis = dist(temp,closest);
		if (dis < stepLength)
		{
			return temp;
		}
		Utils::Point<T> next;

		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		return next;
	}


	template <class T>
	bool DRRT<T>::checkPoint(Utils::Point<T> next)
	{
		return userCheck(next);
	}

	template <class T>
	float DRRT<T>::cost(Utils::Point <T> point){


		Utils::Point <T> parent;
		parent = getParent(point);
		if (point == parent)
		{
			if (point == root)
			{
				return 0;
			}
			return 9999;
		}

		else{
			float costValue;
			costValue = cost(parent) + dist(parent,point);
			// costMap[point] = costValue;
			return costValue;
		}
	}

	template<class T>
	std::vector< Utils::Point<T> > DRRT<T>::getChild(Utils::Point<T> point){
		std::vector< Utils::Point<T> > child;
		int numBucket = findBucket(point);
		std::vector<int> list;
		list.push_back(numBucket);

		// only searching one layer around newRoot
		if (numBucket%(int)numX == 0)
		{
			list.push_back(numBucket + 1);
		}
		else if(numBucket % (int)numX == (int)numX - 1)
			list.push_back(numBucket - 1);
		else{
			list.push_back(numBucket + 1);
			list.push_back(numBucket - 1);
		}
		
		if (numBucket/(int)numX == 0)
		{
			list.push_back(numBucket + (int)numX);	
		}

		else if(numBucket/(int)numX == (int)numY - 1){
			list.push_back(numBucket - (int)numX);
		}

		else{
			list.push_back(numBucket + (int)numX);
			list.push_back(numBucket - (int)numX);
		}

		for (int i = 0; i < list.size(); ++i)
		{
			int numBucket = list[i];
			for (int j = 0; j < bucket[numBucket].second.size(); ++j)
			{
				int id = bucket[numBucket].second[j];
				if (tree[id].second == point)
				{
					child.push_back(tree[id].first);
				}
			}
		}

		return child;
	};


	template <class T>
	Utils::Point<T> DRRT<T>::getParent(Utils::Point<T> cur)
	{
		int numBucket = findBucket(cur);
		// iterate over cur point bucket
		for(int i=0; i<bucket[numBucket].second.size() ;i++)
		{
			int id = bucket[numBucket].second[i];
			if(tree[id].first==cur){
				return tree[id].second;
			}
		}
	}

	template <class T>
	double DRRT<T>::dist(Utils::Point<T> a,Utils::Point<T> b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}


	template<class T>
	void DRRT<T>::reWireRoot(Utils::Point <T> newRoot){
		// unordered_map<Utils::Point<T>, float> costMap;
		std::vector<Utils::Point<T> > path = getPointsOnPath(newRoot);

		std::vector<int> ids;
		for (int i = 0; i < path.size(); ++i)
		{
			int numBucket = findBucket(path[i]);
			for (int j = 0; j < bucket[numBucket].second.size(); ++j)
			{	
				int id = bucket[numBucket].second[j]; 
				if (path[i] ==tree[id].first)
				{
					ids.push_back(id);
					break;
				}
			}
		}
		if (path.size() != ids.size())
		{
			path.erase(path.begin());
		}
		cout<<"\n"<<path.size()<<","<<ids.size();
		tree[ids[0]] = pair< Utils::Point<T> , Utils::Point<T> > (path[0],path[0]);
		root = path[0];
		for (int i = 1; i < ids.size(); ++i)
		{
			int id = ids[i];
			tree[id] = std::pair< Utils::Point<T>, Utils::Point<T> > (path[i],path[i - 1]);
		}

		// New Root of tree is "nearest point" to given point if point is not in tree
		// std::vector<Utils::Point<T> > temppath = getPointsOnPath(startPoint);

		std::vector<Utils::Point<T> > nodes;
		nodes.push_back(path[0]);

		float start = clock();
		int ptr = 0;
		cout<<"\n";
		while(((clock() - start)/CLOCKS_PER_SEC) < timeOut && ptr < nodes.size())
		{
			int numBucket = findBucket(nodes[ptr]);
			
			int myId = -1;
			for (int i = 0; i < bucket[numBucket].second.size(); ++i)
			{
				int id = bucket[numBucket].second[i];
				if (tree[id].first == nodes[ptr])
				{
					myId = id;
					break;
				}
			}

			std::vector<int> list;		//buckets adjacent to rewiring point
			std::vector<int> ids;		//id of nodes in adjacent buckets
			list.push_back(numBucket);

			// only searching one layer around newRoot
			if (numBucket%(int)numX == 0)
			{
				list.push_back(numBucket + 1);
			}
			else if(numBucket % (int)numX == (int)numX - 1)
				list.push_back(numBucket - 1);
			else{
				list.push_back(numBucket + 1);
				list.push_back(numBucket - 1);
			}
			
			if (numBucket/(int)numX == 0)
			{
				list.push_back(numBucket + (int)numX);	
			}

			else if(numBucket/(int)numX == (int)numY - 1){
				list.push_back(numBucket - (int)numX);
			}

			else{
				list.push_back(numBucket + (int)numX);
				list.push_back(numBucket - (int)numX);
			}

			//list contains bucket id for all buckets surrounding newroot
			for (int i = 0; i < list.size(); ++i)
			{
				int numBucket = list[i];
				for (int j = 0; j < bucket[numBucket].second.size(); ++j)
				{
					int id = bucket[numBucket].second[j];
					if (id != myId)
					{
						ids.push_back(id);
					}
				}
			}

			float score = cost(nodes[ptr]);


			for (int i = 0; i < ids.size(); ++i)
			{
				int id = ids[i];
				if (dist(tree[id].first,tree[myId].first) < stepLength)
				{
					if ((cost(tree[id].first) + dist(tree[id].first, tree[myId].first)) < score)
					{
						tree[myId] = pair< Utils::Point<T>, Utils::Point<T> > (tree[myId].first, tree[id].first);
						score = cost(tree[id].first) + dist(tree[id].first, tree[myId].first);
					}
				}
				

				bool flag = false;
				for (int j = 0; j < nodes.size(); ++j)
				{
					if (nodes[j] == tree[id].first)
					{
						flag = true;
						break;
					}
				}

				if (!flag)
				{
					nodes.push_back(tree[id].first);
				}
			}
			ptr++;
		}
		cout<<ptr;
	}

}

