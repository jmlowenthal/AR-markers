#include <opencv2/opencv.hpp>
#include <iostream>
#include "utils.h"

using namespace cv;
using namespace std;

int main_qr()
{
	VideoCapture webcam;
	webcam.open(0);

	if (!webcam.isOpened())	{
		cerr << "Error: Cannot find camera" << endl;
		return -1;
	}

	Mat frame;
	webcam >> frame;
	if (frame.empty()) {
		cerr << "Error: Cannot get image from camera" << endl;
		return -1;
	}

	Mat gray(frame.size(), frame.type());
	Mat otsu(frame.size(), frame.type());
	Mat edges(frame.size(), frame.type());
	Mat traces(frame.size(), CV_8UC3);

	Mat qr, qr_raw, qr_gray, qr_otsu;
	qr_raw = Mat::zeros(100, 100, CV_8UC3);
	qr = Mat::zeros(100, 100, CV_8UC3);
	qr_gray = Mat::zeros(100, 100, CV_8UC1);
	qr_otsu = Mat::zeros(100, 100, CV_8UC1);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	namedWindow("Camera");
	namedWindow("Trace");
	namedWindow("QR");
	namedWindow("Vision");

	moveWindow("Camera", 50, 50);
	moveWindow("Trace", 60, 60);
	moveWindow("QR", 70, 70);
	moveWindow("Vision", 80, 80);

	char key = -1;
	while (key < 0)
	{
		traces = Scalar(0, 0, 0);

		webcam >> frame;

		cvtColor(frame, gray, CV_RGB2GRAY);
		threshold(gray, otsu, 0, 0xff,cv::THRESH_OTSU);

//#define OTSU
#ifdef OTSU
		imshow("Vision", otsu);
		Canny(otsu, edges, 100, 200);
#else
		imshow("Vision", gray);
		Canny(gray, edges, 100, 200);
#endif

		findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		// Moments and centres of mass
		//vector<Moments> mu(contours.size());
		vector<Point2f> mc(contours.size());

		for (unsigned int i = 0; i < contours.size(); ++i) {
			//mu[i] = moments(contours[i]);
			//mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			Moments m = moments(contours[i]);
			mc[i] = Point2f(m.m10 / m.m00, m.m01 / m.m00);
			circle(traces, mc[i], 3, Scalar(128, 0, 128));
		}

		// Find orientation markers
		unsigned int A, B, C;
		unsigned int markerCount = 0;
		for (unsigned int i = 0; i < contours.size() && markerCount < 3; ++i) {
			unsigned int k = i;
			unsigned int c = 0;

			while (hierarchy[k][2] != -1) {
				k = hierarchy[k][2];
				++c;
			}

			if (hierarchy[k][2] != -1) ++c;

			if (c >= 5) {
				if (markerCount == 0) A = i;
				else if (markerCount == 1) B = i;
				else if (markerCount == 2) C = i;
				++markerCount;
			}
		}

		// Find code orientation
		if (markerCount > 2) {					// Maybe we have multiple marker sets found
			double AB = sqdist(mc[A], mc[B]);
			double BC = sqdist(mc[B], mc[C]);
			double CA = sqdist(mc[C], mc[A]);

			// Which is the hypotenuse?
			unsigned int outlier, med1, med2;
			if (AB > BC && AB > CA) {
				outlier = C; med1 = A; med2 = B;
			}
			else if (BC > AB && BC > CA) {
				outlier = A; med1 = B; med2 = C;
			}
			else { // CA > AB && CA > BC
				outlier = B; med1 = C; med2 = A;
			}

			// Determine the direction the code is facing
			bool vertical = false;
			double dist = perpdist(mc[med1], mc[med2], mc[outlier]);
			double slope = gradient(mc[med1], mc[med2], vertical);

			Orientation orientation;
			unsigned int bottom, right;
			if (vertical) {
				bottom = med1;
				right = med2;
				orientation = NORTH;
			}
			else if (slope < 0 && dist < 0)		// Orientation - North
			{
				bottom = med1;
				right = med2;
				orientation = NORTH;
			}
			else if (slope > 0 && dist < 0)		// Orientation - East
			{
				right = med1;
				bottom = med2;
				orientation = EAST;
			}
			else if (slope < 0 && dist > 0)		// Orientation - South			
			{
				right = med1;
				bottom = med2;
				orientation = SOUTH;
			}

			else if (slope > 0 && dist > 0)		// Orientation - West
			{
				bottom = med1;
				right = med2;
				orientation = WEST;
			}

			putText(traces, "outlier", mc[outlier], FONT_HERSHEY_PLAIN, 1, Scalar(0xff, 0xff, 0xff));
			putText(traces, "bottom", mc[bottom], FONT_HERSHEY_PLAIN, 1, Scalar(0xff, 0xff, 0xff));
			putText(traces, "right", mc[right], FONT_HERSHEY_PLAIN, 1, Scalar(0xff, 0xff, 0xff));

			String card[] = {"North", "East", "South", "West"};
			cout << card[orientation] << endl;

			// Find QR code
			if (outlier < contours.size() && right < contours.size() && bottom < contours.size()) {
				//double areaTop = contourArea(contours[outlier]);
				//double areaRight = contourArea(contours[right]);
				//double areaBottom = contourArea(contours[bottom]);
				// if (areaTop > 10 && areaRight > 10 && areaBottom > 10){

				vector<Point2f> L, M, O, tempL, tempM, tempO;
				Point2f N;

				getVertices(contours, outlier, slope, tempL);
				getVertices(contours, right, slope, tempM);
				getVertices(contours, bottom, slope, tempO);

				sequenceCorners(orientation, tempL, L);
				sequenceCorners(orientation, tempM, M);
				sequenceCorners(orientation, tempO, O);

				circle(traces, M[1], 10, Scalar(255, 0, 128));
				circle(traces, M[2], 10, Scalar(255, 0, 128));
				circle(traces, O[3], 10, Scalar(255, 0, 128));
				circle(traces, O[2], 10, Scalar(255, 0, 128));

				if (!getIntersectionPoint(M[1], M[2], O[3], O[2], N)) continue;

				vector<Point2f> src, dst;

				src.push_back(L[0]);
				src.push_back(M[1]);
				src.push_back(N);
				src.push_back(O[3]);

				dst.push_back(Point2f(0, 0));
				dst.push_back(Point2f(qr.cols, 0));
				dst.push_back(Point2f(qr.cols, qr.rows));
				dst.push_back(Point2f(0, qr.rows));

				qr_raw = Mat::zeros(100, 100, CV_8UC3);
				qr = Mat::zeros(100, 100, CV_8UC3);
				qr_gray = Mat::zeros(100, 100, CV_8UC1);
				qr_otsu = Mat::zeros(100, 100, CV_8UC1);

				Mat warp = getPerspectiveTransform(src, dst);
				warpPerspective(frame, qr_raw, warp, Size(qr.cols, qr.rows));

				copyMakeBorder(qr_raw, qr, 10, 10, 10, 10, BORDER_CONSTANT, Scalar(0xff, 0xff, 0xff));
				cvtColor(qr, qr_gray, CV_RGB2GRAY);
				threshold(qr_gray, qr_otsu, 0, 0xff, cv::THRESH_OTSU);

				drawContours(frame, contours, outlier, Scalar(0xff, 0, 0), 2, 8, hierarchy, 0);
				drawContours(frame, contours, right, Scalar(0, 0xff, 0), 2, 8, hierarchy, 0);
				drawContours(frame, contours, bottom, Scalar(0, 0, 0xff), 2, 8, hierarchy, 0);

#ifdef _DEBUG
				drawContours(traces, contours, outlier, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
				drawContours(traces, contours, right, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
				drawContours(traces, contours, bottom, Scalar(255, 0, 100), 1, 8, hierarchy, 0);

				// Draw points (4 corners) on Trace image for each Identification marker	
				circle(traces, L[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, L[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, L[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, L[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				circle(traces, M[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, M[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, M[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, M[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				circle(traces, O[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, O[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, O[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, O[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				circle(traces, mc[outlier], 2, Scalar(255, 0, 100));
				circle(traces, mc[right], 2, Scalar(255, 0, 100));
				circle(traces, mc[bottom], 2, Scalar(255, 0, 100));

				// Draw point of the estimated 4th Corner of (entire) QR Code
				circle(traces, N, 2, Scalar(255, 255, 255), -1, 8, 0);

				// Draw the lines used for estimating the 4th Corner of QR Code
				line(traces, M[1], N, Scalar(0, 0, 255), 1, 8, 0);
				line(traces, O[3], N, Scalar(0, 0, 255), 1, 8, 0);
#endif

				//}
			}
		}

		imshow("Camera", frame);
		imshow("Trace", traces);
		imshow("QR", qr);

		key = cv::waitKey(10);
	}
	return 0;
}

int main_ar() {
	VideoCapture webcam;
	webcam.open(0);

	if (!webcam.isOpened()) {
		cerr << "Error: Cannot find camera" << endl;
		return -1;
	}

	Mat frame;
	webcam >> frame;
	if (frame.empty()) {
		cerr << "Error: Cannot get image from camera" << endl;
		return -1;
	}

	Mat traces(frame.size(), frame.type());
	Mat gray(frame.size(), frame.type());
	Mat edges(frame.size(), frame.type());

	Mat marker_raw, marker_gray, marker_thres;
	marker_raw = Mat::zeros(Size(100, 100), CV_8UC3);
	marker_gray = Mat::zeros(Size(100, 100), CV_8UC3);
	marker_thres = Mat::zeros(Size(100, 100), CV_8UC3);
	
	int key = -1;
	while (key < 0) {
		webcam >> frame;

		traces = Mat::zeros(frame.size(), frame.type());

		cvtColor(frame, gray, CV_RGB2GRAY);
		Canny(gray, edges, 150, 300);

		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		// Look for marker outline border
		drawContours(traces, contours, -1, Scalar(0, 0xff, 0), 1);
		for (unsigned int i = 0; i < contours.size(); ++i) {
			// contour[i] and contour[i] grandchild need to be squares
			unsigned int child = hierarchy[i][2];
			if (child == -1) continue;

			unsigned int grandchild = hierarchy[child][2];
			if (grandchild == -1) continue;

			vector<Point> outer;
			approxPolyDP(contours[i], outer, arcLength(contours[i], true) * 0.02, true);
			if (outer.size() != 4 || !isContourConvex(outer)) continue;

			vector<Point> inner;
			approxPolyDP(contours[grandchild], inner, arcLength(contours[grandchild], true) * 0.02, true);
			if (inner.size() != 4 || !isContourConvex(inner)) continue;

			// Found one
			polylines(traces, outer, true, Scalar(0xff, 0, 0), 2);
			polylines(traces, inner, true, Scalar(0, 0, 0xff), 2);
			
			marker_raw = Mat::zeros(200, 200, CV_8UC3);
			marker_gray = Mat::zeros(200, 200, CV_8UC3);
			marker_thres = Mat::zeros(200, 200, CV_8UC3);

			vector<Point2f> src;
			src.push_back(outer[0]);
			src.push_back(outer[1]);
			src.push_back(outer[2]);
			src.push_back(outer[3]);

			vector<Point2f> dst;
			dst.push_back(Point(0, 0));
			dst.push_back(Point(marker_raw.cols, 0));
			dst.push_back(Point(marker_raw.cols, marker_raw.rows));
			dst.push_back(Point(0, marker_raw.rows));

			Mat warp = getPerspectiveTransform(src, dst);
			warpPerspective(frame, marker_raw, warp, marker_raw.size());
			cvtColor(marker_raw, marker_gray, CV_RGB2GRAY);
			threshold(marker_gray, marker_thres, 0, 0xff, THRESH_OTSU);

			vector<Point2f> newinner;
			newinner.push_back(inner[0]);
			newinner.push_back(inner[1]);
			newinner.push_back(inner[2]);
			newinner.push_back(inner[3]);

			perspectiveTransform(newinner, newinner, warp);
			Rect rect = boundingRect(newinner);
			double gridsize = (2*(marker_thres.cols + marker_thres.rows) - 2 * (rect.width + rect.height)) / 8;
			double effectivesize = (marker_thres.cols * gridsize) / (marker_thres.cols - gridsize);
			for (unsigned int y = 5 * effectivesize / 2; y < marker_thres.cols - 2 * effectivesize; y += effectivesize) {
				for (unsigned int x = 5 * effectivesize / 2; x < marker_thres.rows - 2 * effectivesize; x += effectivesize) {
					Scalar colour = marker_thres.at<uchar>(x, y);
					cout << (colour.val[0] / 255 ? " " : "x");
					ellipse(marker_thres, Point(x, y), Size(1, 1), 0, 0, 360, Scalar(128, 0, 0));
				}
				cout << endl;
			}

			rectangle(marker_thres, rect, Scalar(128, 128, 128), 2);

			cout << endl << endl;
			imshow("Marker", marker_thres);
		}

		imshow("Camera", frame);
		imshow("Traces", traces);

		key = waitKey(20);
	}
}

int main() {
#ifdef QR
	return main_qr();
#else
	return main_ar();
#endif
}