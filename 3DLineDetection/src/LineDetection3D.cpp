


// label == index, region == patch
// result regions: obtained regions after cluster and merging

#include "LineDetection3D.h"
#include <omp.h>

#include "CommonFunctions.h"
#include "Timer.h"

using namespace std;
using namespace cv;

LineDetection3D::LineDetection3D()
{
}

LineDetection3D::~LineDetection3D()
{
}

void LineDetection3D::run( PointCloud<double> &data, int k, std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines, std::vector<double> &ts  )
{
	this->pointData = data;
	this->pointNum = data.pts.size();
	this->k = k;

	// step1: point cloud segmentation
	double totalTime = 0.0;
	CTimer timer;
	char msg[1024];

	timer.Start();
	cout<<endl<<endl;
	cout<<"Step1: Point Cloud Segmentation ..."<<endl;
	std::vector<std::vector<int> > regions; // result regions
	pointCloudSegmentation( regions ); // get result regions from point cloud
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Point Cloud Segmentation Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	// step2: plane based 3D line detection
	timer.Start();
	cout<<"Step2: Plane Based 3D LineDetection ..."<<endl;
	planeBased3DLineDetection( regions, planes ); // (in, out)
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Plane Based 3D LineDetection Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	// step3: post processing
	timer.Start();
	cout<<"Step3: Post Processing ..."<<endl;
	postProcessing( planes, lines );
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Post Processing Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	printf("Total Time: %lf.\n\n", totalTime);
}


void LineDetection3D::pointCloudSegmentation( std::vector<std::vector<int> > &regions )
{
	cout<<"----- Normal Calculation ..."<<endl;
	PCAFunctions pcaer;
	pcaer.Ori_PCA( this->pointData, this->k, this->pcaInfos, this->scale, this->magnitd );
	
	cout<<"----- Region Growing ..."<<endl;
	// threshold of direction deviation to judge whether locate in same region
	double thAngle = 15.0/180.0*CV_PI; // 15° rad; 
	regionGrow( thAngle, regions ); // (input threshold, output regions)

	// step3: region merging
	cout<<"----- Region Merging ..."<<endl;
	double thAnglePatch = thAngle; // angle threshold to merge plane
	regionMerging( thAnglePatch, regions );
}


void LineDetection3D::regionGrow( double thAngle, std::vector<std::vector<int> > &regions )
{
	double thNormal = cos(thAngle); // threshold of normal deviation to judge whether locate in same region

	// sort according to the curvature of points
	std::vector<std::pair<int,double> > idxSorted( this->pointNum );
	for ( int i=0; i<this->pointNum; ++i )
	{
		idxSorted[i].first = i;
		idxSorted[i].second = pcaInfos[i].lambda0;
	}
	// sorted ascendantly according to their curvature lambda
	std::sort( idxSorted.begin(), idxSorted.end(), [](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second < rhs.second; } );

	// get the initial clusters
	// this value is empirically defined to calculate scale(according to article) and to get cluster
	double percent = 0.9; 
	// num of points that should be traversed to calculate scale and get cluster
	int idx = int(this->pointNum*percent); 
	// whether this point is merged into a region, 0(hasn't been merged) & 1(has been merged)
	std::vector<int> isUsed( this->pointNum, 0 ); 
	// 
	for ( int i=0; i<idx; ++i )
	{
		int idxStrater = idxSorted[i].first; // index of this point
		if ( isUsed[idxStrater] ) // if this point has been merged into a region, continue
		{ 
			continue; 
		}
		cv::Matx31d normalStarter = pcaInfos[idxStrater].normal; // direction of this point
		double xStrater = pointData.pts[idxStrater].x; // coordinate of this point
		double yStrater = pointData.pts[idxStrater].y;
		double zStrater = pointData.pts[idxStrater].z;
		// distance threshold to judge whether locate in same region
		double thRadius2 = pow(50*pcaInfos[idxStrater].scale, 2); 

		std::vector<int> clusterTemp;
		clusterTemp.reserve(10000); // capacity and size
		clusterTemp.push_back( idxStrater ); 
		int count = 0;
		// 
		while( count < clusterTemp.size() )
		{
			int idxSeed = clusterTemp[count]; // last added point as seed
			cv::Matx31d normalSeed = pcaInfos[idxSeed].normal;
			double thOrtho = pcaInfos[idxSeed].scale;

			// point cloud collection (add all suitable neighbors to cluster)
			int num = pcaInfos[idxSeed].idxAll.size();
			for( int j = 0; j < num; ++j )
			{
				int idxCur = pcaInfos[idxSeed].idxAll[j]; // index of this neighbor
				if (isUsed[idxCur]) // if this point has been merged
				{
					continue;
				}

				// judgement1: normal deviation
				cv::Matx31d normalCur = pcaInfos[idxCur].normal;
				// direction deviation of this neighbor and this point
				double normalDev = abs(normalCur.val[0] * normalStarter.val[0] + normalCur.val[1] * normalStarter.val[1] + normalCur.val[2] * normalStarter.val[2]);
				// direction deviation of this neighbor and this seed
				//double normalDev = abs(normalCur.val[0] * normalSeed.val[0] + normalCur.val[1] * normalSeed.val[1] + normalCur.val[2] * normalSeed.val[2]);
				if (normalDev < thNormal) // direction deviation larger, normalDev samller
				{
					continue;
				}

				// judgement2: orthogonal distance between this point and this neighbor (Manhattan)
				double dx = pointData.pts[idxCur].x - xStrater;
				double dy = pointData.pts[idxCur].y - yStrater;
				double dz = pointData.pts[idxCur].z - zStrater;
				double dOrtho = abs(dx * normalCur.val[0] + dy * normalCur.val[1] + dz * normalCur.val[2]);
				if (dOrtho > thOrtho) // too far
				{
					continue;
				}

				// judgement3: parallel distance between this point and this neighbor (Euclidean)
				double dPara = dx*dx + dy*dy + dz*dz;
				if (dPara > thRadius2) 
				{
					continue;
				}

				clusterTemp.push_back( idxCur );
				isUsed[idxCur] = 1;
			} // endfor: traverse all neighbors of one seed
			count ++;
		} // endwhile: have gotten cluster

		if ( clusterTemp.size() > 30 ) // save if this cluster is large enough
		{
			regions.push_back( clusterTemp );
		}
		else // discard if cluster is too small 
		{
			for (int j=0; j<clusterTemp.size(); ++j) // release all points in this cluster
			{
				isUsed[clusterTemp[j]] = 0; 
			}
		}
	} // endfor:
}

void LineDetection3D::regionMerging( double thAngle, std::vector<std::vector<int> > &regions )
{
	double thRegionSize = 600000; // size threshold of a result region 

	// step1: plane fitting via PCA for each region
	std::vector<PCAInfo> patches; // save information of all regions in this cloud
	patches.resize( regions.size() );

	#pragma omp parallel for
	for ( int i=0; i<regions.size(); ++i ) // traverse every regions
	{
		int pointNumCur = regions[i].size(); // num of points in this region
		std::vector<std::vector<double> > pointDataCur(pointNumCur); // save all points in this region
		for ( int j=0; j<pointNumCur; ++j )
		{
			pointDataCur[j].resize(3);
			pointDataCur[j][0] = this->pointData.pts[regions[i][j]].x;
			pointDataCur[j][1] = this->pointData.pts[regions[i][j]].y;
			pointDataCur[j][2] = this->pointData.pts[regions[i][j]].z;
		}

		PCAFunctions pcaer;
		// (intput pointdata, output pca patch)
		pcaer.PCASingle( pointDataCur, patches[i] );

		patches[i].idxAll = regions[i];
		double scaleAvg = 0.0;
		for ( int j=0; j<patches[i].idxIn.size(); ++j )
		{
			
			// patches[i].idxIn[j], which == j, is index in this region
			int idx = regions[i][patches[i].idxIn[j]]; // index in this cloud of ( i-th region, j-th point ); 
			patches[i].idxIn[j] = idx;
			scaleAvg += pcaInfos[idx].scale;
		}
		scaleAvg /= patches[i].idxIn.size();
		patches[i].scale = 5.0 * scaleAvg; // 5.0 empirical
	}

	// get the patch label of each point
	std::vector<int> label( this->pointNum, -1 );
	#pragma omp parallel for
	for ( int i=0; i<regions.size(); ++i )
	{
		for ( int j=0; j<regions[i].size(); ++j )
		{
			int id = regions[i][j]; // index in this cloud
			label[id] = i;
		}
	}

	// step2: find adjacent patches
	std::vector<std::vector<int> > patchAdjacent( patches.size() );
	#pragma omp parallel for
	for ( int i=0; i<patches.size(); ++i ) // traverse every region in this cloud
	{
		std::vector<int> patchAdjacentTemp; // save labels of regions that are merged
		std::vector<std::vector<int> > pointAdjacentTemp; // save points in several regions
		for ( int j=0; j<patches[i].idxIn.size(); ++j ) // traverse every point in this region
		{
			int id = patches[i].idxIn[j]; // index of this point in this cloud
			for ( int m=0; m<pcaInfos[id].idxIn.size(); ++m ) // traverse points used for pca
			{
				int idPoint = pcaInfos[id].idxIn[m]; // index (in this cloud) of the point used for pca
				int labelPatch = label[idPoint]; // label of region
				// this point belongs to this region; this point doesn't belong to any region
				if ( labelPatch == i || labelPatch < 0 ) 
				{
					continue;
				}

				bool isNeighbor = false;
				for ( int n=0; n<pcaInfos[idPoint].idxIn.size(); ++n )
				{
					if ( pcaInfos[idPoint].idxIn[n] == id )
					{
						isNeighbor = true;
					}
				}
				if ( ! isNeighbor )
				{
					continue;
				}

				// accept the patch as a neighbor
				bool isIn = false;
				int n = 0;
				for ( n=0; n<patchAdjacentTemp.size(); ++n )
				{
					if ( patchAdjacentTemp[n] == labelPatch ) // this region/patch has been merged in
					{
						isIn = true;
						break;
					}
				}

				if ( isIn )
				{
					pointAdjacentTemp[n].push_back( idPoint ); // put this point into current merged region
				}
				else
				{
					patchAdjacentTemp.push_back( labelPatch );

					std::vector<int> temp;
					temp.push_back( idPoint );
					pointAdjacentTemp.push_back( temp );
				}
			} // endfor: has traversed all points used for pca for a point in this region
		} // endfor: has traversed all points in this region

		// repetition removal
		for ( int j=0; j<pointAdjacentTemp.size(); ++j )
		{
			// ascendantly
			std::sort(pointAdjacentTemp[j].begin(), pointAdjacentTemp[j].end());  
			// 对有序的容器重新排列, 将第一次出现的元素从前往后排, 其他重复出现的元素依次排在后面
			// 返回迭代器, 迭代器指向的是重复元素的首地址
			vector<int>::iterator new_end = unique(pointAdjacentTemp[j].begin(), pointAdjacentTemp[j].end());
			pointAdjacentTemp[j].erase(new_end, pointAdjacentTemp[j].end());

			if ( pointAdjacentTemp[j].size() >= 3 ) // points in this merged region/patch are enough
			{
				patchAdjacent[i].push_back( patchAdjacentTemp[j] );
			}
		}
	} // endfor: has traversed all regions and know which regions/patches should be merged

	// try to merge adjacent patch
	regions.clear();
	// whether this patch has been merged; 0(not merged) & 1(merged)
	std::vector<int> mergedIndex( patches.size(), 0 ); 
	for ( int i=0; i<patches.size(); ++i ) // traverse every patch
	{
		// just merge to get one region in a loop; comments below are just about a loop
		if ( mergedIndex[i] ) 
		{
			continue;
		}
		
		int idxStarter = i; // index of this patch
		cv::Matx31d normalStarter = patches[idxStarter].normal;
		cv::Matx31d ptStarter = patches[idxStarter].planePt; // coordinate of center point

		std::vector<int> patchIdx;
		patchIdx.push_back( idxStarter );

		int count       = 0; // num of merged regions 
		int totalPoints = 0; // num of points that have been merged
		bool isEnough = false;
		while ( count < patchIdx.size() )
		{
			int idxSeed = patchIdx[count];
			cv::Matx31d normalSeed = patches[idxSeed].normal;
			cv::Matx31d ptSeed = patches[idxSeed].planePt;
			double thOrtho = patches[idxSeed].scale;

			for ( int j=0; j<patchAdjacent[idxSeed].size(); ++j ) // traverse regions that will be mergd
			{
				int idxCur = patchAdjacent[idxSeed][j]; // index of region

				if ( mergedIndex[idxCur] )
				{
					continue;
				}

				cv::Matx31d normalCur = patches[idxCur].normal;
				cv::Matx31d ptCur = patches[idxCur].planePt;

				// plane angle deviation and distance
				double devAngle = 0.0;
				double devDis   = 0.0;
				double thDev    = 0.0;

				// Manhattan distance
				cv::Matx31d ptVector1 = ptCur - ptStarter; // distance between region to merge and start region
				cv::Matx31d ptVector2 = ptCur - ptSeed   ; // distance between region to merge and last merged region
				devAngle = acos( normalStarter.val[0] * normalCur.val[0] + normalStarter.val[1] * normalCur.val[1] + normalStarter.val[2] * normalCur.val[2] );
				//devDis = abs( normalSeed.val[0] * ptVector2.val[0] + normalSeed.val[1] * ptVector2.val[1] + normalSeed.val[2] * ptVector2.val[2] );
				devDis = abs( normalStarter.val[0] * ptVector1.val[0] + normalStarter.val[1] * ptVector1.val[1] + normalStarter.val[2] * ptVector1.val[2] );

				if ( min( devAngle, fabs( CV_PI - devAngle ) ) < thAngle && devDis < thOrtho )
				{
					patchIdx.push_back( idxCur );
					mergedIndex[idxCur] = 1; // indicate that this region has been merged

					totalPoints += patches[idxCur].idxAll.size();
					if ( totalPoints > thRegionSize )
					{
						isEnough = true;
						break;
					}
				}
			} // endfor: we have merged adjacent regions of seed region

			if ( isEnough )
			{
				break;
			}
			count ++;
		} // endwhile: we have get a result region

		// create a new cluster
		std::vector<int> patchNewCur;
		for ( int j=0; j<patchIdx.size(); ++j )
		{
			int idx = patchIdx[j];

			for ( int m=0; m<patches[idx].idxAll.size(); ++m )
			{
				patchNewCur.push_back( patches[idx].idxAll[m] );
			}
		}

		// 
		if (patchNewCur.size() > 100)
		{
			regions.push_back( patchNewCur );
		}
		
	} // endfor: we have merged all region
}

void LineDetection3D::planeBased3DLineDetection( std::vector<std::vector<int> > &regions, std::vector<PLANE> &planes )
{
	double thAngle = 10.0/180.0*CV_PI;
	double thLineLength = 8*this->scale; // ???
	int numPatches = regions.size(); 

	// step1: fitting 3D plane via PCA
	std::vector<PCAInfo> patches(numPatches);
	#pragma omp parallel for
	for ( int i=0; i<numPatches; ++i )
	{
		int pointNumCur = regions[i].size();
		std::vector<std::vector<double> > pointDataCur(pointNumCur);
		for ( int j=0; j<pointNumCur; ++j )
		{
			pointDataCur[j].resize(3);
			pointDataCur[j][0] = this->pointData.pts[regions[i][j]].x;
			pointDataCur[j][1] = this->pointData.pts[regions[i][j]].y;
			pointDataCur[j][2] = this->pointData.pts[regions[i][j]].z;
		}

		PCAFunctions pcaer;
		// (intput pointdata, output pca patch)
		pcaer.PCASingle( pointDataCur, patches[i] );

		patches[i].idxAll = regions[i];
		for ( int j=0; j<patches[i].idxIn.size(); ++j )
		{
			int idx = patches[i].idxIn[j];
			patches[i].idxIn[j] = regions[i][idx];
		}
	} // endfor: get every patch

	// step2: 3D line detection
	planes.resize(patches.size());
	#pragma omp parallel for
	for(int i=0; i<patches.size(); ++i)
	{
		// A. 3D-2D Projection: project the 3d point onto the plane coordinate
		std::vector<cv::Point2d> pts2d; // 2d coordinate of 
		std::vector<double> ptScales; // save point's scale
		
		// we need 2d coordinate system. false(haven't set coordinate system), true(have set)
		bool initialized = false; 
		cv::Mat_<double> vX, vY;
		cv::Mat_<double> planePt = (cv::Mat_<double>(3,1) << patches[i].planePt.val[0], patches[i].planePt.val[1], patches[i].planePt.val[2]);
		cv::Mat_<double> normal  = (cv::Mat_<double>(3,1) << patches[i].normal.val[0], patches[i].normal.val[1], patches[i].normal.val[2]);

		for(int j=0; j<patches[i].idxAll.size(); ++j)
		{
			int id = patches[i].idxAll[j];
			cv::Mat_<double> pt3d = (cv::Mat_<double>(3,1) << pointData.pts[id].x, pointData.pts[id].y, pointData.pts[id].z );

			cv::Mat_<double> v3d = pt3d - planePt; // vector from plane pt to point
			cv::Mat_<double> vOrtho = v3d.dot(normal) * normal; // vector between point and plane is perpendicular to plane
			cv::Mat_<double> vPlane = v3d - vOrtho; // projection of v3d on plane
			cv::Mat_<double> ptPlane = planePt + vPlane; // coordinate of projection of point on plane 

			if(!initialized) // if we haven't set coordinate system, use projection of first point to set.
			{
				vX = vPlane * 1.0/(cv::norm(vPlane)); // direction from plane pt to projection point, length == 1
				vY = vX.cross(normal); 
				vY = vY * 1.0/cv::norm(vY);
				initialized = true;
			}
			if( initialized ) // calculate 2d coordinate in plane
			{
				double x = vPlane.dot(vX);
				double y = vPlane.dot(vY);
				pts2d.push_back(cv::Point2d(x,y));
				ptScales.push_back(pcaInfos[id].scale);
			}
		} // endfor: have calculated 2d coordinate of every point in this patch


		// A. 3D-2D Projection: get the side length of the grid cell
		// we will get a binary image by deviding points into grid cells and calculate their index of row and column
		// grid cell larger, image smaller
		double gridSideLength = 0; // the side length of the grid cell
		// ascendantly
		std::sort( ptScales.begin(), ptScales.end(), [](const double& lhs, const double& rhs) { return lhs < rhs; } );
		int idxNinety = min( int(double(ptScales.size()) * 0.9), int(ptScales.size()-1) );
		gridSideLength = ptScales[idxNinety] * 0.75; // reference to article

		// A. 3D-2D Projection: get the binary image of the plane
		double xmin, ymin, xmax, ymax; // the plane's boundary 

		// we can get calculate size of image from plane. 
		int margin = 0; // we should ensure background is a little larger than image from plane 

		// better to consider it as background image, which contains only 0. 
		// Then point will be added into this mask.
		cv::Mat mask; 

		// if we get an image, it's true
		bool isok = maskFromPoint( pts2d, gridSideLength, xmin, ymin, xmax, ymax, margin, mask );
		if ( !isok )
		{
			continue;
		}

		// B. 2D Line Detection
		// ignore lines whose size is less than threshold 
		int thLineLengthPixel = max(thLineLength/gridSideLength,10.0); 
		// every element: all line segments in a contour
		// all     < contour   < line      < poinr     > > >
		std::vector<std::vector<std::vector<cv::Point2d> > > lines2d;
		// (input image, input length threshold, output line segments)
		lineFromMask( mask, thLineLengthPixel, lines2d );
		if (!lines2d.size())
		{
			continue;
		}

		// C. 2D-3D Projection
		planes[i].scale = gridSideLength;
		for ( int m=0; m<lines2d.size(); ++m ) 
		{
			std::vector<std::vector<cv::Point3d> > temp; // save straight lines in a contour
			for (int n=0; n<lines2d[m].size(); ++n)
			{
				// Manhattan length of line segment in pixel image
				double length = abs(lines2d[m][n][1].x-lines2d[m][n][0].x) + abs(lines2d[m][n][1].y-lines2d[m][n][0].y);
				if ( length < thLineLengthPixel )
				{
					continue;
				}

				// coordinate in image without margin
				lines2d[m][n][0].x = (lines2d[m][n][0].x - margin) * gridSideLength + xmin;
				lines2d[m][n][0].y = (lines2d[m][n][0].y - margin) * gridSideLength + ymin;

				lines2d[m][n][1].x = (lines2d[m][n][1].x - margin) * gridSideLength + xmin;
				lines2d[m][n][1].y = (lines2d[m][n][1].y - margin) * gridSideLength + ymin;

				// 3d coordinate
				cv::Mat_<double> xs = lines2d[m][n][0].x * vX; 
				cv::Mat_<double> ys = lines2d[m][n][0].y * vY;
				cv::Mat_<double> pts = planePt + xs + ys;

				cv::Mat_<double> xe = lines2d[m][n][1].x * vX;
				cv::Mat_<double> ye = lines2d[m][n][1].y * vY;
				cv::Mat_<double> pte = planePt + xe + ye;

				std::vector<cv::Point3d> line3dTemp(2);
				line3dTemp[0] = cv::Point3d(pts(0), pts(1), pts(2));
				line3dTemp[1] = cv::Point3d(pte(0), pte(1), pte(2));

				temp.push_back( line3dTemp );
			} // endfor: in a contour
			if (temp.size())
			{
				planes[i].lines3d.push_back(temp);
			}
		} // endfor: process all contours in a patch/region/plane
	} // endfor: process all 
}

bool LineDetection3D::maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, 
                                     double &xmin, double &ymin, double &xmax, double &ymax, 
																		 int &margin, cv::Mat &mask )
{
	xmin = 10000000;
	ymin = 10000000;
	xmax = -xmin;	
	ymax = -ymin;
	for (int i=0; i<pts2d.size(); ++i)
	{
		if(pts2d[i].x < xmin) { xmin = pts2d[i].x; }
		if(pts2d[i].x > xmax) { xmax = pts2d[i].x; }

		if(pts2d[i].y < ymin) { ymin = pts2d[i].y; }
		if(pts2d[i].y > ymax) { ymax = pts2d[i].y; }
	}

	margin = 4;
	int cols = (xmax-xmin) / radius + 2*margin; // ensure image is little larger than plane
	int rows = (ymax-ymin) / radius + 2*margin;
	if ( cols < 10 || rows < 10 ) // this plane is too samll
	{
		return false;
	}

	mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0)); 
	for (int i=0; i<pts2d.size(); ++i) // calculate row and column of every point 
	{
		int xInt = int((pts2d[i].x-xmin)/radius+0.5+margin);
		int yInt = int((pts2d[i].y-ymin)/radius+0.5+margin);
		mask.at<uchar>(yInt,xInt) = 255; // set white if here is a point
	}
	return true;
}

void LineDetection3D::lineFromMask( cv::Mat &mask, int thLineLengthPixel, 
                                    std::vector<std::vector<std::vector<cv::Point2d> > > &lines )
{
	lines.clear();

	// get mask image via dilate and erode
	cv::Mat mask2;
	// remove isolate points, around which no pixel exist(depend on kernel)
	// make image contain only 0 & 255(1), which can be considered as a binary image
	cv::dilate(mask, mask2, cv::Mat()); 
	// back to original pixel distribution
	cv::erode(mask2, mask2, cv::Mat());

	// A. contours
	double thLength = thLineLengthPixel; // ignore lines whose size is less than this threshold 
	
	std::vector<std::vector<cv::Point> > contours;  
	// Vec4i 是 Vec<int,4> 的别名, 定义了一个 “向量内每一个元素包含了 4 个 int 型变量” 的向量
	// hierarchy[i][0] ~ hierarchy[i][3], 分别表示第 i 个轮廓的后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号
	// 如果当前轮廓没有对应的后一个 轮廓、前一个轮廓, 父轮廓或内嵌轮廓的话, 则hierarchy[i][0] ~hierarchy[i][3]的相应位被设置为默认值 -1
	std::vector<cv::Vec4i> hierarchy;
	// https://blog.csdn.net/laobai1015/article/details/76400725
	// (in image, out contour, out index of contour, int in 定义轮廓的检索模式, int in 定义轮廓的近似方法)
	cv::findContours(mask2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// B. line fitting from the contours
	for ( int i=0; i<contours.size(); ++i )
	{
		if ( contours[i].size() < 4*thLength  ) // if this contour is too short
		{
			continue;
		}

		std::vector<std::vector<cv::Point2d> > lineTemp;
		// find and save all line segments in this contour
		// (in unsed, in unused, in contour, in length threshold, out line segments from a contour)
		LineFunctions::lineFitting( mask2.rows, mask2.cols, contours[i], thLength, lineTemp );
		lines.push_back(lineTemp);
	} // endfor: have extracted line segments from every contours
}


void LineDetection3D::postProcessing( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines )
{
	// step1: plane line regularization
	outliersRemoval( planes );
	// now varible planes only contains structural planes, unstructural planes have been removed

	// step2: line merging
	// (in, in|out) result line saved in varible lines
	lineMerging( planes, lines );
}

void LineDetection3D::outliersRemoval( std::vector<PLANE> &planes )
{
	double thCosAngleIN = cos(12.5/180.0*CV_PI);  // angle threshold to judge whether inlier
	double thCosAngleNEW = cos(30.0/180.0*CV_PI); // angle thresholg to judge whether start a new line
	double thNonStructPlaneRatio = 0.3;           // 
	double thAngle = 12.5;                        // angle threshold to remove outlier
	double thCosAngleParal = cos(thAngle/180.0*CV_PI);
	double thCosAngleOrtho = cos((90.0-thAngle)/180.0*CV_PI);
	double thNonStructLineRatio = 10;
	double thStructPlane = 60*this->scale;

	// indicate whether this plane is structural, 1(structural) & 0(unstructural)
	std::vector<int> isPlaneGood(planes.size(), 0);
	#pragma omp parallel for
	for (int i=0; i<planes.size(); ++i) // traverse plane
	{
		if (!planes[i].lines3d.size())
		{
			continue;
		}

		// step1: remove non-structural planes
		std::vector<double> lengthsAll;
		std::vector<cv::Mat> orientsAll;
		std::vector<std::pair<int, double> > lineInfos;
		std::vector<std::vector<double> > lengths(planes[i].lines3d.size());
		std::vector<std::vector<cv::Mat> > orients(planes[i].lines3d.size());

		double totalLength = 0.0;
		int count = 0;
		for (int m=0; m<planes[i].lines3d.size(); ++m) // traverse all contours
		{
			lengths[m].resize(planes[i].lines3d[m].size());
			orients[m].resize(planes[i].lines3d[m].size());
			for (int n=0; n<planes[i].lines3d[m].size(); ++n) // traverse all line segments
			{
				cv::Mat orientTemp = cv::Mat(planes[i].lines3d[m][n][1] - planes[i].lines3d[m][n][0]);
				double lengthTemp = cv::norm(orientTemp);
				lengthsAll.push_back(lengthTemp);
				lengths[m][n] = lengthTemp;

				orientTemp *= 1.0/lengthTemp; // norm(same direction, length == 1)
				orientsAll.push_back(orientTemp);
				orients[m][n] = orientTemp;   // 

				std::pair<int, double> lineInfoTemp(count, lengthTemp);
				lineInfos.push_back(lineInfoTemp);

				totalLength += lengthTemp;
				count ++;
			} // endfor: have traversed all line segments
		} // endfor: have traversed all contours in this plane
		// descendant
		std::sort( lineInfos.begin(), lineInfos.end(), [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) { return lhs.second > rhs.second; } );

		std::vector<cv::Mat> clusterOrient; // direction, length == 1
		std::vector<std::pair<int, double> > clusterInfos; // <index of lines, length>
		for (int j=0; j<lineInfos.size(); ++j) // traverse all line segments in this plane
		{
			int id = lineInfos[j].first; // index of line segment in this plane
			double length = lineInfos[j].second;

			if (!clusterInfos.size()) // if no cluster, push first pair
			{
				clusterInfos.push_back(std::pair<int, double>(clusterInfos.size(), length));
				clusterOrient.push_back(orientsAll[id]);
				continue;
			}

			bool isIn = false; // indicate whether is inlier; false(outlier) & true(inlier)
			double cosValueMin = 100; // max angle deviation
			for (int m=0; m<clusterInfos.size(); ++m) // 
			{
				double cosValue = abs(orientsAll[id].dot(clusterOrient[m]));
				if ( cosValue < cosValueMin )
				{
					cosValueMin =  cosValue;
				}
				if (cosValue > thCosAngleIN) // in same direction
				{
					// 
					clusterInfos[m].second += length;
					isIn = true;
					break;
				}
			}

			if (!isIn && cosValueMin < thCosAngleNEW) // angle deviation > angle threshold to start a new line
			{
				clusterInfos.push_back(std::pair<int, double>(clusterInfos.size(), length));
				clusterOrient.push_back(orientsAll[id]);
				continue;
			}
		} // endfor: 

		double scaleCur = max(this->scale,planes[i].scale);
		if ( clusterInfos.size() > 1)
		{
			double LStruct =  clusterInfos[0].second + clusterInfos[1].second;
			if( LStruct < thNonStructPlaneRatio*totalLength || LStruct < thStructPlane ) // ???
			{
				continue; // remove non-structural plane
			}
		}

		// step2: remove non-structural lines
		PLANE planeNew;
		planeNew.scale = planes[i].scale;
		//double scaleCur = planes[i].scale;
		double thNonStructLineLength = scaleCur*thNonStructLineRatio;
		for (int m=0; m<planes[i].lines3d.size(); ++m) // traverse all planes
		{
			int numLines = planes[i].lines3d[m].size(); // num of lines in this plane

			double lengthTotal = 0.0;
			for (int n=0; n<numLines; ++n) // total length in this plane
			{
				lengthTotal += lengths[m][n];
			}

			double ratioStruct = 0.0;
			double lengthStruct = 0.0;
			std::vector<int> isStruct(numLines, 0);
			if (numLines > 1)
			{
				// judge if the contour is structural
				std::vector<int> idxOrthoPara;
				for (int n=0; n<numLines-1; ++n) // 
				{
					int id1 = n;
					int id2 = (n+1)%numLines; // a loop: if n == num Lines - 1, result will be 0 (the first one)

					double cosAngle = abs(orients[m][id1].dot(orients[m][id2]));
					if (cosAngle > thCosAngleParal || cosAngle < thCosAngleOrtho) // (parallel || orthotic)
					{
						idxOrthoPara.push_back(id1);
						idxOrthoPara.push_back(id2);
					}
				}

				if (idxOrthoPara.size())
				{
					// structural ratio; descentdant
					std::sort( idxOrthoPara.begin(), idxOrthoPara.end(), [](const int& lhs, const int& rhs) { return lhs > rhs; } );

					int idTemp = idxOrthoPara[0]; // indedx of line
					isStruct[idTemp] = 1;
					lengthStruct = lengths[m][idTemp];
					for (int n=0; n<idxOrthoPara.size(); ++n)
					{
						if (idxOrthoPara[n] != idTemp)
						{
							lengthStruct += lengths[m][idxOrthoPara[n]];
							idTemp = idxOrthoPara[n];
							isStruct[idTemp] = 1;
						}
					}

					ratioStruct = lengthStruct/lengthTotal;
				}
			}

			std::vector<std::vector<cv::Point3d> > contourTemp;
			for (int n=0; n<numLines; ++n)
			{
				double thLengthTemp = 0.0; // threshold for this line
				if (isStruct[n])
				{
					if(ratioStruct>=0.75) 
					{
						thLengthTemp = thNonStructLineLength;
					}
					else if (ratioStruct>=0.5) 
					{
						thLengthTemp = 2*thNonStructLineLength;
					}
					else 
					{
						thLengthTemp = 4*thNonStructLineLength;
					}
				}
				else
				{
					thLengthTemp = 4*thNonStructLineLength;
				}

				if (lengths[m][n] > thLengthTemp)
				{
					contourTemp.push_back(planes[i].lines3d[m][n]);
				}
			} // endfor: have traversed all lined in this plane
			if (contourTemp.size())
			{
				planeNew.lines3d.push_back(contourTemp);
			}
		} // endfor: have selected lines for every plane

		if (planeNew.lines3d.size())
		{
			planes[i] = planeNew;
			isPlaneGood[i] = 1;
		}
	} // endfor: have traversed all planes to remove outlier

	//
	std::vector<PLANE> planesNew;
	for (int i=0; i<isPlaneGood.size(); ++i)
	{
		if (isPlaneGood[i])
		{
			planesNew.push_back(planes[i]);
		}
	}
	planes = planesNew;
}

void LineDetection3D::lineMerging( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines )
{
	double thGapRatio = 20;  // ???
	double thMergeRatio = 6; // 
	double thDisHyps = 0.1;  // threshold of distance difference (reference to article)

	// get all the lines
	std::vector<double> lineScales; // save scale for every line
	for (int i=0; i<planes.size(); ++i) // traverse all planes
	{
		for (int m=0; m<planes[i].lines3d.size(); ++m) // traverse all contours in this plane
		{
			for (int n=0; n<planes[i].lines3d[m].size(); ++n) // traverse all lines in this contour
			{
				lines.push_back(planes[i].lines3d[m][n]);
				lineScales.push_back(planes[i].scale);
			}
		}
	}

	// get the parameters of each 3d line
	std::vector<std::vector<double> > lineParas(lines.size()) ;   // vector, ???, , length
	std::vector<std::pair<int, double> > lineInfos(lines.size()); // save index and length of line segments
	for ( int i=0; i<lines.size(); ++i )
	{
		cv::Mat v(lines[i][1]-lines[i][0]); // vector of line segment
		double length = cv::norm(v);
		v *= 1.0/length; // unit vector

		// middle point of line segment
		cv::Mat ptmid((lines[i][1]+lines[i][0])*0.5); 
		// the distance from original point to line segment(reference to article)
		cv::Mat d = v.cross(ptmid)*(1.0/this->magnitd); 

		// get the latitude of the line, longitude is not stable ???
		double latitude = asin(abs(v.at<double>(2)));

		// the length of the line
		lineParas[i].resize(6);
		lineParas[i][0] = v.at<double>(0);       
		lineParas[i][1] = v.at<double>(1);       
		lineParas[i][2] = v.at<double>(2);
		lineParas[i][3] = latitude;   
		lineParas[i][4] = cv::norm(d); 
		lineParas[i][5] = length; 

		lineInfos[i] = std::pair<int,double>(i, length);
	} // endfor: have traverse all line segments in this cloud and calculated parameters 
	// descendant
	std::sort( lineInfos.begin(), lineInfos.end(), [](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second > rhs.second; } );

	// build grid with latitude
	double precision = 6.0/180.0*CV_PI; // size of grid
	int laSize = CV_PI/2.0/precision; // == 15 num of grids
	std::vector<std::vector<int > > grid(laSize);
	std::vector<int> gridIndex(lineParas.size());
	for ( int i=0; i<lineParas.size(); ++i ) // devide line segments into grids
	{
		int la = lineParas[i][3]/precision;
		grid[la].push_back(i);
		gridIndex[i] = la;
	}

	// line merging
	std::vector<bool> isUsed(lines.size(), 0); // indicate whether merged; 1(merged) & 0(unmerged)
	std::vector<std::vector<cv::Point3d> > linesNew; // result after merging
	for ( int i=0; i<lineInfos.size(); ++i ) // traverse all line segments
	{
		int id0 = lineInfos[i].first;
		if ( isUsed[id0] ) // if this line has been merged
		{
			continue;
		}
		isUsed[id0] = 1;

		double lineScale = max(lineScales[id0], this->scale); // ??? why max
		double vx0 = lineParas[id0][0], vy0 = lineParas[id0][1], vz0 = lineParas[id0][2]; // vector
		double d0 = lineParas[id0][4], length0 = lineParas[id0][5]; // distance; length
		cv::Point3d pts0 = lines[id0][0], pte0 = lines[id0][1]; // end points

		// get the merging hypotheses
		std::vector<int> idHyps;
		for (int j=-1; j<=1; ++j) // traverse all grids
		{
			int latemp = gridIndex[id0]+j;
			int la = (latemp+laSize)%laSize;
			for ( int m=0; m<grid[la].size(); ++m ) // traverse all line in this grid
			{
				int idTemp = grid[la][m];
				if (abs(lineParas[idTemp][4]-d0) < thDisHyps)
				{
					idHyps.push_back(idTemp);
				}
			}
		} // endfor: have found hyps for this line segment

		// try merging
		for (int j=0; j<idHyps.size(); ++j)
		{
			int id1 = idHyps[j];
			if ( isUsed[id1] ) // if this line has been merged
			{
				continue;
			}

			cv::Point3d pts1 = lines[id1][0], pte1 = lines[id1][1];
			double length1 = lineParas[id1][5];

			// judge the distance between two line
			cv::Point3d v1 = pts0 - pts1; // start0 - start1
			double disNormal1 = v1.x*vx0 + v1.y*vy0 + v1.z*vz0;
			cv::Point3d vOrtho1 = v1 - disNormal1*cv::Point3d(vx0, vy0, vz0);
			double disOrtho1 = sqrt(vOrtho1.x*vOrtho1.x + vOrtho1.y*vOrtho1.y + vOrtho1.z*vOrtho1.z);

			cv::Point3d v2 = pts0 - pte1; // start0 - end1
			double disNormal2 = v2.x*vx0 + v2.y*vy0 + v2.z*vz0;
			cv::Point3d vOrtho2 = v2 - disNormal2*cv::Point3d(vx0, vy0, vz0);
			double disOrtho2 = sqrt(vOrtho2.x*vOrtho2.x + vOrtho2.y*vOrtho2.y + vOrtho2.z*vOrtho2.z);

			if ( disOrtho1 > thMergeRatio*lineScale || disOrtho2 > thMergeRatio*lineScale )
			{
				continue;
			}

			// judge the overlapping ratio of two line
			cv::Point3d d1 = pts0 - pts1, d2 = pts0 - pte1, d3 = pte0 - pts1, d4 = pte0 - pte1;
			double dis1 = sqrt(d1.x*d1.x + d1.y*d1.y + d1.z*d1.z);
			double dis2 = sqrt(d2.x*d2.x + d2.y*d2.y + d2.z*d2.z);
			double dis3 = sqrt(d3.x*d3.x + d3.y*d3.y + d3.z*d3.z);
			double dis4 = sqrt(d4.x*d4.x + d4.y*d4.y + d4.z*d4.z);
			double disMerge = max( max(dis1, dis2), max(dis3, dis4) );

			double gapLength = disMerge - length0 - length1;
			double gapRatio = gapLength / length0;
			if ( gapRatio < 0.1 && gapLength < thGapRatio*lineScale )
			{
				// update start and end point of line id0
				if (gapRatio > 0)
				{
					if (dis1 == disMerge)
					{
						double disNormal = d1.x*vx0 + d1.y*vy0 + d1.z*vz0;
						lines[id0][1] = pts0 - disNormal*cv::Point3d(vx0, vy0, vz0);
					}
					else if (dis2 == disMerge)
					{
						double disNormal = d2.x*vx0 + d2.y*vy0 + d2.z*vz0;
						lines[id0][1] = pts0 - disNormal*cv::Point3d(vx0, vy0, vz0);
					}
					else if (dis3 == disMerge)
					{
						double disNormal = d3.x*vx0 + d3.y*vy0 + d3.z*vz0;
						lines[id0][0] = pte0 - disNormal*cv::Point3d(vx0, vy0, vz0);
					}
					else
					{
						double disNormal = d4.x*vx0 + d4.y*vy0 + d4.z*vz0;
						lines[id0][0] = pte0 - disNormal*cv::Point3d(vx0, vy0, vz0);
					}
				}

				isUsed[id1] = 1;
			}
		} // endfor: have merged all suitable lines

		linesNew.push_back(lines[id0]); // save result line after merging and no more line will be added to result line 
	} // endfor: have found and merged line for every line

	lines = linesNew;
}
