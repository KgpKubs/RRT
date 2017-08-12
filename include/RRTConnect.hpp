#include <iostream>
#include "./RRT.hpp"
#include "./RRT_implementation.hpp"
#include "./utils.hpp"

using namespace std;
namespace rrt
{
	template <class T>
	void RRTConnect<T>::setEndPoints(Utils::Point<T> start, Utils::Point<T> end)
	{
		startPoint=start;
		endPoint=end;
	}

	template <class T>
	void RRTConnect<T>::setHalfDimensions(double a, double b)
	{
		halfDimensionX=a;
		halfDimensionY=b;
	}

	template <class T>
	void RRTConnect<T>::setOrigin(Utils::Point<T> pt)
	{
		origin=pt;
	}

	template <class T>
	std::vector<Utils::Point<T> > RRTConnect<T>::getPointsOnPath()
	{
		return pathPoints;
	}

	template <class T>
	void RRTConnect<T>::setStepLength(double value)
	{
		stepLength=value;
	}

	template <class T>
	void RRTConnect<T>::setMaxIterations(int value)
	{
		maxIterations=value;
	}

	template <class T>
	bool RRTConnect<T>:: plan()
	{
		int count=0;
		int check=0;
		treeA.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (startPoint,startPoint));
		treeB.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (endPoint,endPoint));

		int start_time=clock();
		while( check < maxIterations)
		{
			//std::cout<<"In Planning Loop"<<std::endl;
			Utils::Point<T> next;


			if (check%2 == 0)
			{
				mainTree = treeA;
				supportTree = treeB;
			}
			else{
				mainTree = treeB;
				supportTree = treeA;
			}

			if(treeComplete()==true)
			{
				std::cout<<"Tree complete!!"<<std::endl;
				// completeTree(treeA);
				int end_time=clock();
				std::cout<<"Time to generate path = "<<(float)(end_time-start_time)/CLOCKS_PER_SEC<<std::endl;
				generatePath(startPoint,endPoint);
				return true;
			}
			else if(count%biasParameter==0)
			{
				// std::cout<<"Adding Next Biased Point to tree!!"<<std::endl;

				// count=0;
				do{
					if (check%2 == 0)
					{
						next = endPoint;
					}
					else{
						next = startPoint;
					}
				}while(checkPoint(next)!=true);

			}
			else
			{
				// std::cout<<"Adding next point to tree!!"<<std::endl;
				do{

					next = generatePoint(mainTree);
				}while(checkPoint(next)!=true);

			}
			// std::cout<<" Growing Tree next : "<<next.x<<","<<next.y<<std::endl;
			growTree(next);
			count++;
			check++;
			//std::cout<<"check= "<<check<<", count= "<<count<<std::endl;
		}
		std::cout<<"Path Not Found"<<std::endl;
		return false;
	}

	template <class T>

	void RRTConnect<T>::growTree(Utils::Point<T> temp)
	{
		//growing the tree by adding node 
		//std::cout<<"finding parent in tree of size = "<<tree.size()<<std::endl;

		Utils::Point<T> closest;
		closest = findClosestNode(temp,treeA);
		double theta = atan2(temp.y-closest.y,temp.x-closest.x);
		double dis = dist(temp,closest);
		if (dis > stepLength)
		{
		temp.x=closest.x+stepLength*cos(theta);
		temp.y=closest.y+stepLength*sin(theta);
		}
		treeA.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (temp,closest));
		
		closest = findClosestNode(temp,treeB);
		theta = atan2(temp.y - closest.y , temp.x - closest.x);
		dis = dist(temp,closest);
		if (dis > stepLength)
		{
		temp.x=closest.x+stepLength*cos(theta);
		temp.y=closest.y+stepLength*sin(theta);
		}
		treeB.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (temp,closest));
		//std::cout<<"Point Generated = "<<temp.x<<","<<temp.y<<std::endl;
		//std::cout<<"Tree grown"<<std::endl;
		//add pruning code to keep size of tree under control
	}
	

	template <class T>
	void RRTConnect<T>::setCheckPointFunction(bool (*f)(Utils::Point<T>))
	{
		userCheck=f;
	}

	template <class T>
	Utils::Point<T> RRTConnect<T>::findClosestNode(Utils::Point<T> cur,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > >tree)
	{
		double min_dist=INT_MAX;
		Utils::Point<T> closest;
		for(int i=0;i<tree.size();i++)
		{
			////std::cout<<"Itering the tree to find closest Node"<<std::endl;
			if(dist(tree[i].first,cur) < min_dist )
			{
				min_dist=dist(tree[i].first,cur);
				closest=tree[i].first;
			}
		}
		return closest;
	}

	template <class T>
	Utils::Point<T> RRTConnect<T>::generatePoint(std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree)
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


		return temp;
		Utils::Point<T> closest=findClosestNode(temp,tree);
		double theta = atan2(temp.y-closest.y,temp.x-closest.x);
		double dis = dist(temp,closest);
		if (dis < stepLength)
		{
			return temp;
		}
		Utils::Point<T> next;

		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		// std::cout<<"origin,"<<origin.x<<" "<<origin.y<<" "<<temp.x<<" "<<temp.y<<" "<<next.x<<" "<<next.y<<std::endl;
		//std::cout<<"Point Generated = "<<temp.x<<","<<temp.y<<std::endl;
		return next;
	}

	template <class T>
	Utils::Point<T> RRTConnect<T>::generateBiasedPoint(Utils::Point<T> endPoint,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > >tree)
	{
		Utils::Point<T> closest=findClosestNode(endPoint,tree);
		double theta = atan2(endPoint.y-closest.y,endPoint.x-closest.x);

		Utils::Point<T> next;
		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		//std::cout<<"Biased Point Generated = "<<next.x<<","<<next.y<<std::endl;
		return next;
	}

	template <class T>
	bool RRTConnect<T>::checkPoint(Utils::Point<T> next)
	{
		return userCheck(next);
	}

	template <class T>
	bool RRTConnect<T>::treeComplete()
	{
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > conn;
		//std::cout<<"Checking if tree is complete? "<<std::endl;
		//std::cout<<"tree size = "<<tree.size()<<std::endl;
		int min_dist=INT_MAX;	
		for(int i=0;i<treeA.size();i++)
		{
			for (int j = 0; j < treeB.size(); ++j)
			{
				double dis= dist(treeA[i].first,treeB[j].first);
				if(dis < min_dist)
				{	min_dist=dis;
					// std::cout<<treeA.size()<<" "<<treeB.size()<<std::endl;
				}
				if(dis < stepLength )
				{
					connectA = treeA[i].first;
					connectB = treeB[j].first;

					conn.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (connectA,connectB));
					// std::cout<<"distance = "<<dis<<" ";
					// std::cout<<"points "<<treeA[i].first.x<<" "<<treeA[i].first.y;
					// std::cout<<"points "<<treeB[j].first.x<<" "<<treeB[j].first.y;

				}
			}
			
		}
		if (conn.size() )
		{
			double thetaDiffMax = 9999;
			double thetaA,thetaB,thetaDiff,thetaMain;
			int a  = 0;
			for (int i = 0; i < conn.size(); ++i)
			{
				thetaMain = atan2(conn[i].second.y - conn[i].first.y ,conn[i].second.x - conn[i].first.x);
				thetaA = atan2(conn[i].first.y - getParent(conn[i].first,treeA).y, conn[i].first.x - getParent(conn[i].first,treeA).x);
				thetaB = atan2(getParent(conn[i].second,treeB).y - conn[i].second.y , getParent(conn[i].second,treeB).x - conn[i].second.x);
				thetaDiff = abs(abs(thetaMain - thetaA) - abs(thetaB - thetaMain));
				if (thetaDiff < thetaDiffMax)
				{
					thetaDiffMax = thetaDiff;
					a=i;
				}
			}
			connectA = conn[a].first;
			connectB = conn[a].second;
			// cout<<"Found Two Probable Joints\n";
			// cout<<"--------"<<a<<"-------\n";
			return true;
		}
		return false;
	}

	template <class T>
	void RRTConnect<T>::setBiasParameter(unsigned int param)
	{
		biasParameter=param;
	}

	template <class T>
	void RRTConnect<T>::generatePath(Utils::Point<T> first,Utils::Point<T> last)
	{

		Utils::Point<T> cur=connectA;
		while(cur!=startPoint){
			pathPoints.insert(pathPoints.begin(),cur);
			cur = getParent(cur,treeA);
		}

		pathPoints.insert(pathPoints.begin(),cur);

		cur = connectB;

		// std::cout<<first.x<<" "<<first.y<<" "<<startPoint.x<<" "<<startPoint.y<<std::endl;
		while(cur!=endPoint)
		{
			pathPoints.push_back(cur);
			cur = getParent(cur,treeB);
			// Utils::Point<T> parent= getParent(cur);
			// std::cout<<"current : "<<cur.x<<","<<cur.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
			// pathPoints.push_back(parent);
			// cur=parent;
		}
		pathPoints.push_back(cur);
	}

	template <class T>
	Utils::Point<T> RRTConnect<T>::getParent(Utils::Point<T> cur,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > >tree)
	{
		for(int i=0;i<tree.size();i++)
		{
			if(tree[i].first==cur)
				return tree[i].second;
		}
	}

	template <class T>
	double RRTConnect<T>::dist(Utils::Point<T> a,Utils::Point<T> b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
}

