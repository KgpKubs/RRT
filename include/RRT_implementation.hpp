#include <iostream>
#include "./RRT.hpp"
#include "./utils.hpp"

namespace rrt
{
	template <class T>
	void RRT<T>::setEndPoints(Utils::Point<T> start, Utils::Point<T> end)
	{
		startPoint=start;
		endPoint=end;
	}

	template <class T>
	void RRT<T>::setHalfDimensions(double a, double b)
	{
		halfDimensionX=a;
		halfDimensionY=b;
	}

	template <class T>
	void RRT<T>::setOrigin(Utils::Point<T> pt)
	{
		origin=pt;
	}

	template <class T>
	std::vector<Utils::Point<T> > RRT<T>::getPointsOnPath()
	{
		Utils::Point<T> parent = pathPoints[0];
		finalPath.push_back(parent);
		for (int i=1; i<pathPoints.size();i++)
		{
			Utils::Point<T> now = pathPoints[1];
			if(checkPoint(parent,now)!=true)
			{
				parent = pathPoints[i-1];
				i--;
				if (i!=0)
					finalPath.push_back(parent);
			}

		}
		finalPath.push_back(pathPoints[pathPoints.size()-1]);
		return finalPath;
	}

	template <class T>
	void RRT<T>::setStepLength(double value)
	{
		stepLength=value;
	}

	template <class T>
	void RRT<T>::setMaxIterations(int value)
	{
		maxIterations=value;
	}

	template <class T>
	void RRT<T>::setObstacleRadius(int value)
	{
		obstacleradius=value;
	}

	template <class T>
	bool RRT<T>:: plan()
	{
		int count=0;
		int check=0;
		tree.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (startPoint,startPoint));
		while( check < maxIterations)
		{
			//std::cout<<"In Planning Loop"<<std::endl;
			Utils::Point<T> next;
			int arr[]={1};
			std::pair  <Utils::Point <T>,Utils::Point <T> >  mid = treeComplete(arr);
			std::pair  <Utils::Point <T>,int >  both;
			// Utils::Point<T> both;
			if(mid.first!=mid.second)
			{
				std::cout<<"Tree complete!!"<<std::endl;
				int start_time=clock();
				both= findClosestNode(endPoint);
				growTree(both,endPoint);
				int end_time=clock();
				std::cout<<"Time to generate path = "<<end_time-start_time<<std::endl;
				generatePath(startPoint,endPoint);
				return true;
			}
			else if(count%biasParameter==0)
			{
				//std::cout<<"Adding Next Biased Point to tree!!"<<std::endl;
				count=0;
				do{
					next= generateBiasedPoint(1);
					both= findClosestNode(next);
				}while(checkPoint(both.first, next)!=true);
				//std::cout<<" : "<<next.x<<","<<next.y<<std::endl;
			}
			else
			{
				//std::cout<<"Adding next point to tree!!"<<std::endl;
				do{
					next = generatePoint();
					both= findClosestNode(next);
				}while(checkPoint(both.first, next)!=true);
				//std::cout<<" : "<<next.x<<","<<next.y<<std::endl;
			}
			//std::cout<<" Growing Tree next : "<<next.x<<","<<next.y<<std::endl;
			growTree(both, next);
			count++;
			check++;
			//std::cout<<"check= "<<check<<", count= "<<count<<std::endl;
		}
		std::cout<<"Path Not Found"<<std::endl;
		return false;
	}

	template <class T>
	void RRT<T>::growTree(std::pair  <Utils::Point <T>,int > parent,Utils::Point<T> next)
	{
		//growing the tree by adding node
		//std::cout<<"finding parent in tree of size = "<<tree.size()<<std::endl;
	
		// parent= findClosestNode(next).first;
		//std::cout<<"current : "<<next.x<<","<<next.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
		tree.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (next,parent.first));
		//std::cout<<"Tree grown"<<std::endl;
		//add pruning code to keep size of tree under control
	}

	template <class T>
	void RRT<T>::setCheckPointFunction(bool (*f)(Utils::Point<T>))
	{
		userCheck=f;
	}

	template <class T>
	std::pair<Utils::Point<T>, int>  RRT<T>::findClosestNode(Utils::Point<T> cur)
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
		return std::make_pair(closest,1);
	}

	template <class T>
	Utils::Point<T> RRT<T>::generatePoint()
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

		Utils::Point<T> closest=findClosestNode(temp).first;
		double theta = atan2(temp.y-closest.y,temp.x-closest.x);

		Utils::Point<T> next;
		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		//std::cout<<"Point Generated = "<<temp.x<<","<<temp.y<<std::endl;
		return next;
	}

	template <class T>
	Utils::Point<T> RRT<T>::generateBiasedPoint(int which)
	{
		Utils::Point<T> closest=findClosestNode(endPoint).first;
		double theta = atan2(endPoint.y-closest.y,endPoint.x-closest.x);

		Utils::Point<T> next;
		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		//std::cout<<"Biased Point Generated = "<<next.x<<","<<next.y<<std::endl;
		return next;
	}

	template <class T>
	bool RRT<T>::obstacle_here(int x, int y)
	{
		for (int i=0;i<ObstaclePoints.size();i++)
		{
			Utils::Point<T> pratham;
			pratham.x = x; pratham.y = y;
			Utils::Point<T> dwitiya;
			dwitiya.x = ObstaclePoints[i].x; dwitiya.y = ObstaclePoints[i].y;
			if (dist(pratham,dwitiya)<obstacleradius)
				return true;
		}
		return false;
	}


	template <class T>
	bool RRT<T>::checkPoint(Utils::Point<T> parent, Utils::Point<T> next)
	{
		int x1=parent.x, x2=next.x, y1=parent.y,y2=next.y;
		float x=x1, count;
		try
		{
		 // std::cout<<"\nIn try ";
		 // std::cout<<x1<<std::endl;
		 // std::cout<<x2<<std::endl;
		 // std::cout<<y1<<std::endl;
		 // std::cout<<y2<<std::endl;

		        int m=float(y2-y1)/(x2-x1);
		        if (m==0) throw 20;
		        int c=y2-m*x2;
		        count = fabs(1.0/m);
		        // while(1)
		        if (count>1) count=1;
		        if (count<-1) count=-1;
		        if (x2<x1) count*=-1;
		        // std::cout<<"\nm is "<<m<<" and count is: "<<count<<"\n";
		        while (1)
		        {
		            x+=count;
		            int y=m*x+c;
		            if ((count>0 and x>=x2) || (count<0 and x<=x2))
		            {
		                // std::cout<<"Return true from try\n";
		                return true;
		            }
		            else
		            {
		        // std::cout<<"\nm is "<<m<<" and count is: "<<count<<"\n";
		            	// std::cout<<std::endl<<"x: "<<x<<" x2: "<<x2<<std::endl;
		            	// std::cout<<"count: "<<count<<std::endl;
		            }
		            if (obstacle_here(x,y))
		            {
		                // std::cout<<"Return false from try\n";
		                return false;
		            }
		        }
		}
		catch(int e)
		{
		        count=1;
		        int y=y1;
		        if (y2<y1) count*=-1;
		        while (1)
		        {
		            y+=count;
		            if ((count>0 and y>=y2) || (count<0 and y<=y2))
		            {
		            	// std::cout<<"Return true from catch\n";
		                return true;
		            }
		            if (obstacle_here(x,y))
		            {
		            	// std::cout<<"Return false from catch\n";
		                return false;
		            }
		        }
		}
	}

	template <class T>
	std::pair <Utils::Point <T>,Utils::Point <T> > RRT<T>::treeComplete(int arr[1])
	{
		//std::cout<<"Checking if tree is complete? "<<std::endl;
		//std::cout<<"tree size = "<<tree.size()<<std::endl;
		int min_dist=INT_MAX;
		for(int i=0;i<tree.size();i++)
		{
			double dis= dist(tree[i].first,endPoint);
			if(dis < min_dist)
				min_dist=dis;
			if(dis < stepLength )
			{
				return std::make_pair(tree[i].first,endPoint);
			}
		}
		//std::cout<<"Min Distance In this iteration = "<<min_dist<<std::endl;
		return std::make_pair(endPoint,endPoint);
	}

	template <class T>
	void RRT<T>::setBiasParameter(unsigned int param)
	{
		biasParameter=param;
	}

	template <class T>
	void RRT<T>::generatePath(Utils::Point<T> first,Utils::Point<T> last)
	{
		Utils::Point<T> cur=last;
		pathPoints.push_back(cur);
		while(cur!=first || cur!=startPoint)
		{
			Utils::Point<T> parent= getParent(cur);
			//std::cout<<"current : "<<cur.x<<","<<cur.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
			pathPoints.push_back(parent);
			cur=parent;
		}
	}

	template <class T>
	Utils::Point<T> RRT<T>::getParent(Utils::Point<T> cur)
	{
		for(int i=0;i<tree.size();i++)
		{
			if(tree[i].first==cur)
				return tree[i].second;
		}
	}

	template <class T>
	double RRT<T>::dist(Utils::Point<T> a,Utils::Point<T> b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
}
