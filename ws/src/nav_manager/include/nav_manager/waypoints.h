#ifndef WAYPOINT_H_
#define WAYPOINT_H_

struct Waypoint
{
    Waypoint() :
        id(0), x(0), y(0) {}

    Waypoint(int _id, double _x, double _y) :
        id(_id), x(_x), y(_y) {}

    int id;
    double x;
    double y;
};

#endif  // WAYPOINT_H_