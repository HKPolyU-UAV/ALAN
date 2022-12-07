// implementation of DBSCAN 1996 for point cloud clustering
//@pattylo
#include <iostream>

#include <math.h>
#include <cmath>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define UNCLASSIFIED -1
#define NOISE -2


typedef struct pts
{
    double x, y, z;
    int clid;
}pts;

class dbscan
{
    void pts_preprocess(std::vector<Eigen::Vector3d>);
    bool expandcluster(pts currentpoint);
    std::vector<size_t> regionquery(pts currentpoint);
    //return all the indicators in pts_raw
    inline double distance(const pts candidate_core, const pts abritraty_pt);

    std::vector<pts> pts_return;
    double eps_radius;
    int minPoints, clusterid;
    std::chrono::time_point <std::chrono::steady_clock> start, end;
    double ms;

public:
    dbscan(double eps, int minPts)
    {
        this->eps_radius = eps;
        this->minPoints = minPts;
        std::cout<<"instantiate dbscan object..."<<std::endl;
    }
    ~dbscan(){};        
    std::vector<pts> execute(std::vector<Eigen::Vector3d> pts_list);
};

inline double dbscan::distance(const pts candidate_core, const pts abritraty_pt) 
{
    return pow(candidate_core.x - abritraty_pt.x, 2)
            + pow(candidate_core.y - abritraty_pt.y, 2)
            + pow(candidate_core.z - abritraty_pt.z, 2);
}

void dbscan::pts_preprocess(std::vector<Eigen::Vector3d> pts_list)
{
    for(auto stuff : pts_list)
    {
        pts temp;
        temp = {stuff(0), stuff(1), stuff(2), UNCLASSIFIED};
        pts_return.push_back(temp);
    }
}

std::vector<size_t> dbscan::regionquery(pts currentpoint)
{
    std::vector<size_t> returnvalue;
    for(size_t i = 0; i < pts_return.size(); i++)
    {
        if( distance(currentpoint, pts_return[i]) < eps_radius )
        {
            returnvalue.push_back(i);
        }        
    }
    return returnvalue;
}

bool dbscan::expandcluster(pts currentpoint)
{
    std::vector<size_t> seeds = regionquery(currentpoint);
    // return a vector of seeds point where points are within the eps-neighborhood
    if(seeds.size() < minPoints)
    {
        currentpoint.clid = NOISE;
        return false;
    }
    else
    {
        int i = 0, coreindex = 0;
        for(auto id : seeds)
        {
            pts_return[id].clid = clusterid;
            if(pts_return[id].x == currentpoint.x 
                && pts_return[id].y == currentpoint.y
                && pts_return[id].z == currentpoint.z)
            {
                coreindex = i;                
            }
            i++;
        }
        
        seeds.erase(seeds.begin() + coreindex);//seeds.delete(Point)

        int temp_i = 0;;

        for(std::vector<size_t>::size_type i = 0, n = seeds.size(); i<n; i++)
        {
            std::vector<size_t> seedpoint_neighbor = regionquery(pts_return[seeds[i]]);
            if(seedpoint_neighbor.size() >= minPoints)
            {
                for(auto id : seedpoint_neighbor)
                {
                    if(pts_return[id].clid == UNCLASSIFIED || pts_return[id].clid == NOISE)
                    {
                        if(pts_return[id].clid == UNCLASSIFIED)
                        {
                            n = seeds.size();
                            seeds.push_back(id);
                        }
                        pts_return[id].clid = clusterid;  
                                                  
                    }
                }
            }
            temp_i ++;        
        }
        
        return true;
    }    
}

std::vector<pts> dbscan::execute(std::vector<Eigen::Vector3d> pts_list)
{

    start = std::chrono::steady_clock::now();
    
    pts_return.clear();
    pts_preprocess(pts_list);

    clusterid = 1;

    for(auto point : pts_return)
    {
        if(point.clid == UNCLASSIFIED) //0 is defined as unclassified
        { 
            if(expandcluster(point))
            {
                clusterid += 1;
            }                                                                                
        }      
    }

    end = std::chrono::steady_clock::now();
    ms = double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    std::cout<<"clustring time: "<<ms<<" ms"<<std::endl;
    
    return pts_return;
}


