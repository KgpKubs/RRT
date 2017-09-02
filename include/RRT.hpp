#ifndef _RRT_
#define _RRT_

#include <iostream>
#include <bits/stdc++.h>
// #include <unordered_map>
#include "utils.hpp"

namespace rrt
{
	template <class T>
	class RRT
	{
	private:
		double halfDimensionX;
		double halfDimensionY;
		Utils::Point<T> origin;
		Utils::Point<T> startPoint;
		Utils::Point<T> endPoint;
		double stepLength;
		std::vector<Utils::Point<T> > pathPoints;
		int maxIterations;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree; 
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree1; 
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree2; 
		unsigned int biasParameter;
		
	public:
		
		RRT(){};
		RRT(Utils::Point<T> start,Utils::Point<T> end)
		{
			//srand(time(NULL));			
			startPoint=start;
			endPoint=end;
		}
		
		virtual bool plan();
		virtual std::vector<Utils::Point<T> > getPointsOnPath();

		virtual void setEndPoints(Utils::Point<T> start, Utils::Point<T> end);
		virtual void setCheckPointFunction(bool (*f)(Utils::Point<T>));
		virtual void setStepLength(double value);
		virtual void setOrigin(Utils::Point<T> origin);
		virtual void setHalfDimensions(double x,double y);
		virtual void setBiasParameter(unsigned int);
		virtual void setMaxIterations(int);
		//TODO : To be implemented in the derived classes
		// virtual void interpolate();
		// virtual void fitVelocityProfile();
		// virtual void pruneTree();

	private:
		bool (*userCheck)(Utils::Point<T>);
		bool checkPoint(Utils::Point<T> pt);
		Utils::Point<T> generatePoint();
		Utils::Point<T> generateBiasedPoint();
		void growTree(Utils::Point<T>);
		Utils::Point<T> findClosestNode(Utils::Point<T>);
		Utils::Point<T> getParent(Utils::Point<T>);
		bool treeComplete();
		void generatePath(Utils::Point<T> first,Utils::Point<T> last);
		double dist(Utils::Point<T> a,Utils::Point<T> b);
	};

	template <class T>
	class RRTConnect: public RRT<T>{

	public:
		bool plan();
		 std::vector<Utils::Point<T> > getPointsOnPath();

		void setEndPoints(Utils::Point<T> start, Utils::Point<T> end);
		void setCheckPointFunction(bool (*f)(Utils::Point<T>));
		void setStepLength(double value);
		void setOrigin(Utils::Point<T> origin);
		void setHalfDimensions(double x,double y);
		void setBiasParameter(unsigned int);
		void setMaxIterations(int);
	private:
		double halfDimensionX;
		double halfDimensionY;
		Utils::Point<T> origin;
		Utils::Point<T> connectA;
		Utils::Point<T> connectB;
		Utils::Point<T> startPoint;
		Utils::Point<T> endPoint;
		double stepLength;
		std::vector<Utils::Point<T> > pathPoints;
		int maxIterations;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > treeA; 
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > treeB;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > mainTree;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > supportTree; 

		unsigned int biasParameter;
		bool (*userCheck)(Utils::Point<T>);
		bool checkPoint(Utils::Point<T> pt);
		Utils::Point<T> generatePoint(std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > >);
		Utils::Point<T> generateBiasedPoint(Utils::Point<T> ,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > >);
		Utils::Point<T> findClosestNode(Utils::Point<T>,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree);
		void growTree(Utils::Point<T>);
		Utils::Point<T> getParent(Utils::Point<T>,std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree);
		bool treeComplete();
		void generatePath(Utils::Point<T> first,Utils::Point<T> last);
		double dist(Utils::Point<T> a,Utils::Point<T> b);
	};


	template <class T>
	class DRRT: public RRT<T>{
	private:
		std::vector<int> obstacleBucket;
		std::vector<std::pair< int, std::vector <int> > > bucket;
		float numX;
		float numY;
		double halfDimensionX;
		double halfDimensionY;
		Utils::Point<T> origin;
		Utils::Point<T> startPoint;
		Utils::Point<T> endPoint;
		double stepLength;
		std::vector<Utils::Point<T> > pathPoints;
		int maxIterations;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree; 
		unsigned int biasParameter;
		unsigned int BucketSize;
		unsigned int PointsInBucket;
		float timeOut;
		Utils::Point<T> root;

	public:
		bool plan();
		std::vector<Utils::Point<T> > getPath(Utils::Point<T>,Utils::Point<T>);
		std::vector<Utils::Point<T> > getPointsOnPath(Utils::Point<T>);
		std::vector<std::pair<Utils::Point<T>, int > > obstacles;
		void setEndPoints(Utils::Point<T> start, Utils::Point<T> end);
		void setCheckPointFunction(bool (*f)(Utils::Point<T>));
		void setStepLength(double value);
		void setOrigin(Utils::Point<T> origin);
		void setHalfDimensions(double x,double y);
		void setMaxIterations(int);
		void setBucketSize(unsigned int);
		void setPointsInBucket(unsigned int);
		void generateGrid();
		void setTimeOut(float time);

	
	private:
		void reWireRoot(Utils::Point<T> point);
		int findBucket(Utils::Point<T>);
		bool saturation();
		bool collision(Utils::Point<T>);
		bool dense(Utils::Point<T>);
		bool (*userCheck)(Utils::Point<T>);
		bool checkPoint(Utils::Point<T> pt);
		Utils::Point<T> generatePoint();
		Utils::Point<T> findClosestNode(Utils::Point<T>);
		void growTree(Utils::Point<T>);
		Utils::Point<T> getParent(Utils::Point<T>);
		bool treeComplete();
		void generatePath(Utils::Point<T> first,Utils::Point<T> last);
		double dist(Utils::Point<T> a,Utils::Point<T> b);
		float cost(Utils::Point<T> point);
		std::vector< Utils::Point<T> > getChild(Utils::Point<T> point);
	};
}




#endif