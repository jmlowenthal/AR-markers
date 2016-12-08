#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

enum Orientation {
	NORTH, EAST, SOUTH, WEST
};

double sqdist(Point2f a, Point2f b);

double cross(Point2f v1, Point2f v2);

/* Perpendicular distance from the line ab to c
|(y2 - y1)x0 - (x2 - x1)y0 + x2y1 - y2x1|
-----------------------------------------
sqrt((y2 - y1)^2 + (x2 - x1)^2)
*/
double perpdist(Point2f a, Point2f b, Point2f c);

double gradient(Point2f a, Point2f b, bool& vertical);

void getVertices(vector<vector<Point>> contours, int index, float slope, vector<Point2f>& quad);

void sequenceCorners(Orientation o, vector<Point2f> IN, vector<Point2f>& OUT);

bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);