#include "utils.h"

double sqdist(Point2f a, Point2f b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return dx*dx + dy*dy;
}

double cross(Point2f v1, Point2f v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}

#if 0
/* Perpendicular distance from the line ab to c
|(y2 - y1)x0 - (x2 - x1)y0 + x2y1 - y2x1|
-----------------------------------------
sqrt((y2 - y1)^2 + (x2 - x1)^2)
*/
double perpdist(Point2f a, Point2f b, Point2f c) {
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	return /*abs*/(dy*c.x - dx*c.y + b.x*a.y - b.y*a.x) / sqrt(dy*dy + dx*dx);
}
#endif
double perpdist(Point2f L, Point2f M, Point2f J) {
	float a, b, c, pdist;

	a = -((M.y - L.y) / (M.x - L.x));
	b = 1.0;
	c = (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y;

	// Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

	pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
	return pdist;
}


double gradient(Point2f a, Point2f b, bool& vertical) {
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	if (dx == 0) {
		vertical = true;
		return 0;
	}
	else {
		vertical = false;
		return dy / dx;
	}
}

void updateCorner(Point2f P, Point2f ref, float& baselinesq, Point2f& corner) {
	double distsq = sqdist(P, ref);
	if (distsq > baselinesq) {
		baselinesq = distsq;
		corner = P;
	}
}

void getVertices(vector<vector<Point>> contours, int index, float slope, vector<Point2f>& quad) {
	Rect box = boundingRect(contours[index]);
	Point2f M0, M1, M2, M3;
	Point2f A, B, C, D, W, X, Y, Z;

	A = box.tl();
	C = box.br();
	B = Point2f(C.x, A.y);
	D = Point2f(A.x, C.y);

	W = Point2f((A.x + B.x) / 2, A.y);
	X = Point2f(A.x, (B.y + C.y) / 2);
	Y = Point2f((C.x + D.x) / 2, C.y);
	Z = Point2f(D.x, (D.y + A.y) / 2);

	float dmaxsq[4] = {0, 0, 0, 0};
	float pd1 = 0, pd2 = 0;

	if (slope > 5 || slope < -5) {
		for (unsigned int i = 0; i < contours[index].size(); ++i) {
			pd1 = perpdist(C, A, contours[index][i]);
			pd2 = perpdist(B, D, contours[index][i]);

			if (pd1 >= 0 && pd2 >= 0) {
				updateCorner(contours[index][i], W, dmaxsq[1], M1);
			}
			else if (pd1 >= 0 && pd2 <= 0) {
				updateCorner(contours[index][i], X, dmaxsq[2], M2);
			}
			else if (pd1 <= 0 && pd2 <= 0) {
				updateCorner(contours[index][i], Y, dmaxsq[3], M3);
			}
			else if (pd1 <= 0 && pd2 >= 0) {
				updateCorner(contours[index][i], Z, dmaxsq[0], M0);
			}
			else continue;
		}
	}
	else {
		float halfx = (A.x + B.x) / 2;
		float halfy = (A.y + D.y) / 2;

		for (int i = 0; i < contours[index].size(); i++)
		{
			if ((contours[index][i].x <= halfx) && (contours[index][i].y <= halfy)) {
				updateCorner(contours[index][i], C, dmaxsq[2], M0);
			}
			else if ((contours[index][i].x >= halfx) && (contours[index][i].y <= halfy)) {
				updateCorner(contours[index][i], D, dmaxsq[3], M1);
			}
			else if ((contours[index][i].x >= halfx) && (contours[index][i].y >= halfy)) {
				updateCorner(contours[index][i], A, dmaxsq[0], M2);
			}
			else if ((contours[index][i].x <= halfx) && (contours[index][i].y >= halfy)) {
				updateCorner(contours[index][i], B, dmaxsq[1], M3);
			}
		}
	}

	quad.push_back(M0);
	quad.push_back(M1);
	quad.push_back(M2);
	quad.push_back(M3);
}

void sequenceCorners(Orientation o, vector<Point2f> IN, vector<Point2f>& OUT) {
	Point2f M0, M1, M2, M3;
	if (o == NORTH) {
		M0 = IN[0];
		M1 = IN[1];
		M2 = IN[2];
		M3 = IN[3];
	}
	else if (o == EAST) {
		M0 = IN[1];
		M1 = IN[2];
		M2 = IN[3];
		M3 = IN[0];
	}
	else if (o == SOUTH) {
		M0 = IN[2];
		M1 = IN[3];
		M2 = IN[0];
		M3 = IN[1];
	}
	else if (o == WEST) {
		M0 = IN[3];
		M1 = IN[0];
		M2 = IN[1];
		M3 = IN[2];
	}

	OUT.push_back(M0);
	OUT.push_back(M1);
	OUT.push_back(M2);
	OUT.push_back(M3);
}

bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection) {
	Point2f p = a1;
	Point2f q = b1;
	Point2f r(a2 - a1);
	Point2f s(b2 - b1);

	if (cross(r, s) == 0) return false;

	float t = cross(q - p, s) / cross(r, s);
	intersection = p + t*r;

	return true;
}