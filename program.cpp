// program that performs a k-means clustering on a 3D point cloud
// 
// the dimensionality of point cloud can be changed in total_values
// number of iterration is fixed to 100
// 
//  to run:
//  ./program path/to/point/cloud k
// path/to/point/cloud is path to txt file containing 3d points
//  k is number of clusters
// 
//  example: ./program /Users/Bogdanova\ Iva/PROJECTS/Pix4D/exercise/cloud.txt 2

// @ author: IB, June 2016

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <string>	// string, stod
#include <sstream>


using namespace std;



// /----------------------Kmeans ----------
class Point
{
private:
	int id_point, id_cluster;
	vector<double> values;
	int total_values;
	string name;

public:
	Point(int id_point, vector<double>& values, string name = "")
	{
		this->id_point = id_point;
		total_values = values.size();

		for(int i = 0; i < total_values; i++)
			this->values.push_back(values[i]);

		this->name = name;
		id_cluster = -1;
	}

	int getID()
	{
		return id_point;
	}

	void setCluster(int id_cluster)
	{
		this->id_cluster = id_cluster;
	}

	int getCluster()
	{
		return id_cluster;
	}

	double getValue(int index)
	{
		return values[index];
	}

	int getTotalValues()
	{
		return total_values;
	}

	void addValue(double value)
	{
		values.push_back(value);
	}

	string getName()
	{
		return name;
	}
};

// -------------------------------------------------
class Cluster
{
private:
	int id_cluster;
	vector<double> central_values;
	vector<Point> points;

public:
	Cluster(int id_cluster, Point point)
	{
		this->id_cluster = id_cluster;

		int total_values = point.getTotalValues();

		for(int i = 0; i < total_values; i++)
			central_values.push_back(point.getValue(i));

		points.push_back(point);
	}

	void addPoint(Point point)
	{
		points.push_back(point);
	}

	bool removePoint(int id_point)
	{
		int total_points = points.size();

		for(int i = 0; i < total_points; i++)
		{
			if(points[i].getID() == id_point)
			{
				points.erase(points.begin() + i);
				return true;
			}
		}
		return false;
	}

	double getCentralValue(int index)
	{
		return central_values[index];
	}

	void setCentralValue(int index, double value)
	{
		central_values[index] = value;
	}

	Point getPoint(int index)
	{
		return points[index];
	}

	int getTotalPoints()
	{
		return points.size();
	}

	int getID()
	{
		return id_cluster;
	}
};

// --------------------------------------------------
class KMeans
{
private:
	int K; // number of clusters
	int total_values, total_points, max_iterations;
	vector<Cluster> clusters;

	// return ID of nearest center (uses Euclidean distance)
	int getIDNearestCenter(Point point)
	{
		double sum = 0.0, min_dist;
		int id_cluster_center = 0;

		for(int i = 0; i < total_values; i++)
		{
			sum += pow(clusters[0].getCentralValue(i) -
					   point.getValue(i), 2.0);
		}

		min_dist = sqrt(sum);

		for(int i = 1; i < K; i++)
		{
			double dist;
			sum = 0.0;

			for(int j = 0; j < total_values; j++)
			{
				sum += pow(clusters[i].getCentralValue(j) -
						   point.getValue(j), 2.0);
			}

			dist = sqrt(sum);

			if(dist < min_dist)
			{
				min_dist = dist;
				id_cluster_center = i;
			}
		}

		return id_cluster_center;
	}

public:
	KMeans(int K, int total_points, int total_values, int max_iterations)
	{
		this->K = K;
		this->total_points = total_points;
		this->total_values = total_values;
		this->max_iterations = max_iterations;
	}

	void run(vector<Point> & points)
	{
		if(K > total_points)
			return;

		vector<int> prohibited_indexes;

		// choose K distinct values for the centers of the clusters
		for(int i = 0; i < K; i++)
		{
			while(true)
			{
				int index_point = rand() % total_points;

				if(find(prohibited_indexes.begin(), prohibited_indexes.end(),
						index_point) == prohibited_indexes.end())
				{
					prohibited_indexes.push_back(index_point);
					points[index_point].setCluster(i);
					Cluster cluster(i, points[index_point]);
					clusters.push_back(cluster);
					break;
				}
			}
		}

		int iter = 1;

		while(true)
		{
			bool done = true;

			// associates each point to the nearest center
			for(int i = 0; i < total_points; i++)
			{
				int id_old_cluster = points[i].getCluster();
				int id_nearest_center = getIDNearestCenter(points[i]);

				if(id_old_cluster != id_nearest_center)
				{
					if(id_old_cluster != -1)
						clusters[id_old_cluster].removePoint(points[i].getID());

					points[i].setCluster(id_nearest_center);
					clusters[id_nearest_center].addPoint(points[i]);
					done = false;
				}
			}

			// recalculating the center of each cluster
			for(int i = 0; i < K; i++)
			{
				for(int j = 0; j < total_values; j++)
				{
					int total_points_cluster = clusters[i].getTotalPoints();
					double sum = 0.0;

					if(total_points_cluster > 0)
					{
						for(int p = 0; p < total_points_cluster; p++)
							sum += clusters[i].getPoint(p).getValue(j);
						clusters[i].setCentralValue(j, sum / total_points_cluster);
					}
				}
			}

			if(done == true || iter >= max_iterations)
			{
				cout << "Break in iteration " << iter << "\n\n";
				break;
			}

			iter++;
		}

		// shows elements of clusters
		for(int i = 0; i < K; i++)
		{
			int total_points_cluster =  clusters[i].getTotalPoints();

			cout << "Cluster " << clusters[i].getID() + 1 << endl;
			for(int j = 0; j < total_points_cluster; j++)
			{
				cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
				for(int p = 0; p < total_values; p++)
					cout << clusters[i].getPoint(j).getValue(p) << " ";

				string point_name = clusters[i].getPoint(j).getName();

				if(point_name != "")
					cout << "- " << point_name;

				cout << endl;
			}

			cout << "Cluster centroid: ";

			for(int j = 0; j < total_values; j++)
				cout << clusters[i].getCentralValue(j) << " ";

			cout << "\n\n";
		}
	}
};

//  -----------help message-----------------
static void show_usage( char* name)
{
    cerr << "PROGRAM THAT PERFORMS K-MEANS CLUSTERING ON 3D POINT CLOUD"<< "\n"
    	<< "Usage: " << name << " <path> <k>"<< "\n"
    	<< "\t<path>: /path/to/point/cloud" << "\n"
    	<< "\t<k>: number of clusters" << "\n"
        << "Options:\n"
        << "\t--help\t\tShow this help message\n"
        << endl;
}

// -------------------------------------------
// -------------------------------------------
int main(int argc, char *argv[])
{
	if (argc < 3) 
    {
        show_usage(argv[0]);
        return 1;
    }
    for (int i = 1; i < argc; ++i) 
    {
        string arg = argv[i];
        if ((arg == "--help")) 
        {
            show_usage(argv[0]);
            return 0;
        } 
    }
    if (argc == 3)
    {	
	// 	do if correct input arguments
	srand (time(NULL));

	int total_points, total_values, K, max_iterations, has_name;
	
	has_name = 0; // 1 if the data point has a name (string), a string column in txt file 
	max_iterations = 100; 
	total_values = 3; // 3D point (X, Y, Z)		
	
	//  K - number of clusters	(command line input)		 
	K = atoi(argv[2]); // 	reads third argument of the command line as K number of clusters
	
	// 	total_points: the number of points in the cloud (number of lines in the txt file)
	// 	read data points from txt file
	total_points = 0;
	int i = 0;
	
	vector<Point> points;
	string point_name;
	vector<double> values;
	vector<double> temp;
	double d = 0.0;
	
	string line;
//     ifstream myfile("cloud.txt");
    cout << "loads file : " << argv[1] << endl;
	ifstream myfile(argv[1]);
	
    if (myfile.is_open())
    {
    	while ( getline (myfile,line)) 
        {
            stringstream ss(line);
            while (ss >> d)
            {
//             	cout << "d = " << d << endl;
            	temp.push_back(d);
            }			
            // for (int i = 0; i < temp.size (); ++i)
//         		cout << "temp[" << i << "] = " << temp.at(i) << endl;
//         	take only the total_values element from the whole vector				
			vector<double> values(temp.end() - total_values, temp.end());
			// for (int i = 0; i < values.size (); ++i)
//         		cout << "values[" << i << "] = " << values.at(i) << endl;
        	Point p(total_points, values);
			points.push_back(p);
			total_points++;
        }           	
        myfile.close();
    }
    else cout << "Unable to open file";
    
    cout << "number of points in text file: " << total_points << endl;
    
	KMeans kmeans(K, total_points, total_values, max_iterations);
	kmeans.run(points);
	} // close argc == 3
	return 0;
}


