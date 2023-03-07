#ifndef _COMMON_FUNCTIONS_
#define _COMMON_FUNCTIONS_
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "nanoflann.hpp"
#include "utils.h"

using namespace cv;
using namespace std;
using namespace nanoflann;

// process single contour
class LineFunctions
{
public:
	LineFunctions(void){};
	~LineFunctions(void){};

public:
  // row and col unused;
	// 
	static void lineFitting( int rows, int cols, std::vector<cv::Point> &contour, 
	                         double thMinimalLineLength, std::vector<std::vector<cv::Point2d> > &lines );

	// cut contour into line segments and discard some that don't meet condition
	/* (out line segment, in contour, 
	    in start index of current contour, in end index of current contour, 
			in point's deviation to line, in min length)
	*/
	static void subDivision( std::vector<std::vector<cv::Point> > &straightString, std::vector<cv::Point> &contour, 
	                         int first_index, int last_index, double min_deviation, int min_size  );

	// calculate result of line fit
	static void lineFittingSVD( cv::Point *points, int length, std::vector<double> &parameters, double &maxDev );
};

// decribe a point
struct PCAInfo
{
	double lambda0, scale; // to evaluate result pca; 尺度, if neighbors and this point is close, scale will be a small value
	cv::Matx31d normal, planePt; // direction; center coordinate of this plane
	std::vector<int> idxAll, idxIn; // save all index of points used for pca; index of inlier points(reference to article)

	PCAInfo &operator =(const PCAInfo &info)
	{
		this->lambda0 = info.lambda0;
		this->normal = info.normal;
		this->idxIn = info.idxIn;
		this->idxAll = info.idxAll;
		this->scale = scale;
		return *this;
	}
};

class PCAFunctions 
{
public:
	PCAFunctions(void){};
	~PCAFunctions(void){};

	// (point cloud, pca result, 尺度, distance between first point of this cloud and lidar)
	void Ori_PCA( PointCloud<double> &cloud, int k, std::vector<PCAInfo> &pcaInfos, double &scale, double &magnitd );
	// (input pointcloud n*3, output pcainfo) get pca info for every point of this cloud
	void PCASingle( std::vector<std::vector<double> > &pointData, PCAInfo &pcaInfo );

	void MCMD_OutlierRemoval( std::vector<std::vector<double> > &pointData, PCAInfo &pcaInfo );

	double meadian( std::vector<double> dataset );
};

#endif //_COMMON_FUNCTIONS_
