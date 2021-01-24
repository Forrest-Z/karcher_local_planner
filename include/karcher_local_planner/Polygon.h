#ifndef POLYGON_H_
#define POLYGON_H_

#include "karcher_local_planner/karcher_local_planner_node.h"
#include <vector>

#define MIN(x, y) (x <= y ? x : y)
#define MAX(x, y) (x >= y ? x : y)

class Polygon
{
public:
    std::vector<Waypoint> points;

    inline int PointInsidePolygon(const Polygon& polygon,const Waypoint& p)
    {
        int counter = 0;
        int i;
        double xinters;
        Waypoint p1, p2;
        int N = polygon.points.size();
        if(N <= 0) return -1;

        p1 = polygon.points.at(0);
        for(i = 1; i <= N; i++)
        {
            p2 = polygon.points.at(i % N);

            if (p.y > MIN(p1.y, p2.y))
            {
                if (p.y <= MAX(p1.y, p2.y))
                {
                    if (p.x <= MAX(p1.x, p2.x))
                    {
                        if (p1.y != p2.y)
                        {
                            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                            if (p1.x == p2.x || p.x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }

        if (counter % 2 == 0)
            return 0;
        else
            return 1;
    }
};

#endif