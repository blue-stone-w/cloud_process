#ifndef _LINE_DETECTION_H_
#define _LINE_DETECTION_H_
#pragma once

#include "CommonFunctions.h"

struct PLANE
{
	double scale;
	std::vector<std::vector<std::vector<cv::Point3d> > > lines3d;

	PLANE &operator =(const PLANE &info)
	{
		this->scale    = info.scale;
		this->lines3d     = info.lines3d;
		return *this;
	}
};

class LineDetection3D 
{
public:
	LineDetection3D();
	~LineDetection3D();

	void run( PointCloud<double> &data, int k, std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines, std::vector<double> &ts );

	// get growed region
	void pointCloudSegmentation( std::vector<std::vector<int> > &regions );
	// (input regions, output planes)
	void planeBased3DLineDetection( std::vector<std::vector<int> > &regions, std::vector<PLANE> &planes );

	void postProcessing( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines );

	// cluster points 
	void regionGrow( double thAngle, std::vector<std::vector<int> > &regions );
	// merge regions that locate in same plane
	void regionMerging( double thAngle, std::vector<std::vector<int> > &regions );

	bool maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, double &xmin, double &ymin, double &xmax, double &ymax, int &margin, cv::Mat &mask );

	void lineFromMask( cv::Mat &mask, int thLineLengthPixel, std::vector<std::vector<std::vector<cv::Point2d> > > &lines );

	void outliersRemoval( std::vector<PLANE> &planes );

	// (in, out)
	void lineMerging( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines );

public:
	int k;
	int pointNum;
	double scale, magnitd; // distance; the distance between the first point of the input point cloud to the original point
	std::vector<PCAInfo> pcaInfos; // save all points info 
	PointCloud<double> pointData;
};

#endif //_LINE_DETECTION_H_
