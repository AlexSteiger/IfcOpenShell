#include <vector>
#include <iostream>
#include <fstream>
//#include <QString>

#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_straight_skeleton_2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //for getMinMax3D
#include <pcl/ModelCoefficients.h>  //for planar coefficients
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>  // for downsampling
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/surface/mls.h>    //MovingLeastSquares


#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/core/exterior_ring.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

//OCC:
#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS.hxx>
#include "STEPControl_Writer.hxx"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

int main(int argc, char *argv[]) {

	//The Clouds: input, semgmentation, projection, concave
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	
  pcl::PCDWriter PCDwriter;
	
	std::string filename;
	if (!argv[1])
	{
		filename = "../../../PCL_Hydromapper/Samples_PCD/Spundwand.pcd";
		std::cout << "Keine Inputdatei als Argument angegeben.";
		std::cout << filename << " wird aufgerufen.\n";
	}
	if (argv[1])
	{
		std:: cout << "Inputdatei angegeben" << std::endl;
		filename = argv[1];
	}
	
  // Input Cloud
  pcl::PCLPointCloud2 cloud_blob;
  //pcl::io::loadPCDFile ("Spundwand.pcd", cloud_blob);
  pcl::io::loadPCDFile (filename.c_str(), cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  // Some informations about input Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt); 
	int nrpoints = cloud->points.size();
  std::cout << "Dimensions of Input (";
  std::cout << nrpoints << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
  PCDwriter.write ("E0_Input_Cloud.pcd", *cloud, false);
	
  // Statistical outlier removal
	nrpoints = cloud->points.size();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
  sor1.setInputCloud (cloud);
  sor1.setMeanK (50);
  sor1.setStddevMulThresh (3);
  sor1.setNegative (false);
  sor1.filter (*cloud);
	std::cout << "Statistical outlier removal: ";
  std::cout << nrpoints - cloud->points.size () << " outliers removed. ";
  std::cout << cloud->points.size() << " points left" << std::endl; 
  PCDwriter.write ("E1_Cloud_filtered.pcd", *cloud);
	
  // MovingLeastSquares
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
	nrpoints = cloud->points.size();
  std::cout << "Starting Moving Least Squares with " << nrpoints << " points.\n";
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //(<Input_Cloud, Output_Cloud>)
  mls.setInputCloud (cloud);
  mls.setComputeNormals (false);
  mls.setSearchRadius (0.1);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder (3);
  mls.process (*cloud_smoothed);
  std::cout << cloud_smoothed->points.size () << " points left." << std::endl;
  PCDwriter.write ("E2_cloud_smoothed.pcd", *cloud_smoothed);
	
  // Segment the cloud
	double lower_limit = -7;
	double upper_limit = -4;
  std::cout << "Starting segmenting the Cloud from ";
	std::cout << lower_limit << " to " << upper_limit << std::endl;
  pcl::PassThrough<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud_smoothed);
  seg.setFilterFieldName ("z");
  seg.setFilterLimits (lower_limit, upper_limit);
  seg.setFilterLimitsNegative (false);
  seg.filter (*cloud_project);
  PCDwriter.write ("E3_Cloud_segmented.pcd", *cloud_project, false);
	
  // Create a set of planar coefficients (The plane coefficients are: a,b,c,d (ax+by+cz+d=0)) 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0; // x = y = 0
  coefficients->values[2] = 1;                           // z 
  coefficients->values[3] = -maxPt.z;                    // d (Hessesche Normalform)
  
  // Project
  double z = maxPt.z;
  std::cout << "Starting projecting the Cloud to z = " << maxPt.z << std::endl;
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_project);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_project);
  PCDwriter.write ("E4_Cloud_projected.pcd", *cloud_project, false);
	
  // Statistical outlier removal 2
	nrpoints = cloud_project->points.size();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_project);
  sor2.setMeanK (50);
  sor2.setStddevMulThresh (4);
  sor2.setNegative (false);
  sor2.filter (*cloud_project);
	std::cout << "Statistical outlier removal: ";
	std::cout << nrpoints - cloud_project->points.size () << " outliers removed. ";
  std::cout << cloud_project->points.size() << " points left" << std::endl; 
  PCDwriter.write ("E5_Cloud_projected_inliers.pcd", *cloud_project, false);
	
  // Concave Hull
	nrpoints = cloud_project->points.size();
  std::cout << "Construct the Concave Hull with " << nrpoints << "points.\n";
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_project);
  chull.setAlpha (0.05);  //limits the size of the hull semements, smaller->more Detail
  chull.reconstruct (*cloud_project);
  std::cerr << "Concave Hull has " << cloud_project->points.size () << " points.\n";
	PCDwriter.write ("E6_Concave_Hull.pcd", *cloud_project, false);

	// Copy Points from PCL to CGAL
  CGAL::Polygon_2<K> poly ;
	for (size_t j = 0; j < cloud_project->points.size(); ++j)
		poly.push_back(K::Point_2(cloud_project->points[j].x,cloud_project->points[j].y) ) ;
	
	// Create the Straight_skeleton
	boost::shared_ptr<CGAL::Straight_skeleton_2<K>> iss = CGAL::create_interior_straight_skeleton_2(poly);
	
	// Extract the interiour points of the skeleton
	//std::cout << "Faces:" << iss->size_of_faces() << std::endl;
	std::vector<K::Point_2> Pt_Skeleton;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_skeleton (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::PointType punkt;
	for( auto face = iss->faces_begin(); face != iss->faces_end(); ++face ) 
		{
			CGAL::Straight_skeleton_2<K>::Halfedge_const_handle begin = face->halfedge();
			CGAL::Straight_skeleton_2<K>::Halfedge_const_handle edge = begin;
			// Each vertex:
			do {
				if (edge->vertex()->is_skeleton()) 
				{
				punkt.x = edge->vertex()->point().x(); punkt.y = edge->vertex()->point().y(); punkt.z = z;
				cloud_skeleton->push_back(punkt);
				Pt_Skeleton.push_back(edge->vertex()->point());
				}
				edge = edge->next();
			} while (edge != begin);
		}
  PCDwriter.write ("E7_Cloud_Skeleton.pcd", *cloud_skeleton);
	std::cout << "Skeleton has " << cloud_skeleton->points.size() << " points." << std::endl;
	
	// Statistical outlier removal
	nrpoints = cloud_skeleton->points.size();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
  sor3.setInputCloud (cloud_skeleton);
  sor3.setMeanK (10);
  sor3.setStddevMulThresh (3.5);
  sor3.setNegative (false);
  sor3.filter (*cloud_skeleton);
  std::cout << nrpoints - cloud_skeleton->points.size () << " outliers removed. ";
  std::cout << cloud_skeleton->points.size() << " points left" << std::endl; 
	PCDwriter.write ("E8_Cloud_Skeleton_filtered.pcd", *cloud_skeleton);
		
	// Downsample the Cloud with a Voxelgrid filter
	nrpoints = cloud_skeleton->points.size();
	pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud_skeleton);
  vox.setLeafSize (0.001f, 0.001f, 0.001f);
  vox.filter (*cloud_skeleton);
	std::cout << "Downsampling:" << nrpoints - cloud_skeleton->points.size () << " points removed. ";
  std::cout << cloud_skeleton->points.size() << " points left" << std::endl; 
	PCDwriter.write ("E9_Cloud_Skeleton_downsampled.pcd", *cloud_skeleton);
	
	// From PCL-Cloud to Boost-Multipoint
	boost::geometry::model::multi_point<boost_point> boost_mpoint_Skeleton;
	for (size_t j = 0; j < cloud_skeleton->points.size() ; ++j)
		boost::geometry::append(boost_mpoint_Skeleton, boost_point(cloud_skeleton->points[j].x,cloud_skeleton->points[j].y));

	//Create Buffer around the Skeleton with Boost
	std::cout << "creating buffer with " << cloud_skeleton->points.size() << " points" << std::endl;
	const double buffer_distance = 0.02;
	const int points_per_circle = 5;
	typedef double coordinate_type;
	boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
	boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
	boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
	boost::geometry::strategy::buffer::side_straight side_strategy;
	boost::geometry::model::multi_polygon<boost_polygon> bufferMPoly;
	boost::geometry::buffer(boost_mpoint_Skeleton, bufferMPoly,	distance_strategy, side_strategy,
							join_strategy, end_strategy, circle_strategy);
	
	// From Boost-MultiPolygon to PCL-Cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buffer (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::PointType cloud_poly_pkt;
	boost_polygon bufferPoly;
	int current_polygon = 0;
	for(auto itr = bufferMPoly.begin(); itr != bufferMPoly.end(); ++itr)
	{
			for(auto it = boost::begin(boost::geometry::exterior_ring(bufferMPoly[current_polygon])); it != boost::end(boost::geometry::exterior_ring(bufferMPoly[current_polygon])); ++it)
		{
			double x = boost::geometry::get<0>(*it);
			double y = boost::geometry::get<1>(*it);
			cloud_poly_pkt.x = x; cloud_poly_pkt.y = y; cloud_poly_pkt.z = z;
			cloud_buffer->push_back(cloud_poly_pkt);
		}
		current_polygon ++;
	}
	std::cout << "Nr of Points in Buffer-Hull: " << cloud_buffer->points.size() << std::endl;
	PCDwriter.write ("E10_Cloud_Buffer.pcd", *cloud_buffer);
	
	// Concave Hull
	nrpoints = cloud_buffer->points.size();
  std::cout << "Construct the Concave Hull with " << nrpoints << "points.\n";
  pcl::ConcaveHull<pcl::PointXYZ> chull2;
  chull2.setInputCloud (cloud_buffer);
	//Min dist: 0.001
  chull2.setAlpha (0.06);  //limits size of hull semements, smaller->more Detail, 0.01 = 1 cm
  chull2.reconstruct (*cloud_buffer);
  std::cerr << "Concave Hull has " << cloud_buffer->points.size () << " points.\n";
	PCDwriter.write ("E11_Cloud_Buffer_Concave_Hull.pcd", *cloud_buffer, false);
	
	// Statistical outlier removal
	nrpoints = cloud_buffer->points.size();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor4;
  sor4.setInputCloud (cloud_buffer);
  sor4.setMeanK (15);
  sor4.setStddevMulThresh (2);
  sor4.setNegative (false);
  sor4.filter (*cloud_buffer);
  std::cout << nrpoints - cloud_buffer->points.size () << " outliers removed. ";
  std::cout << cloud_buffer->points.size() << " points left" << std::endl; 
	PCDwriter.write ("E12_Cloud_Buffer_filtered.pcd", *cloud_buffer);
	
  //OpenCascade
  // Take the the concave_hull points and make a OCC Polygon with it. 
  // The concave_hull points are ordered clockwise, 
  // so there is no problem constructing the Polygon.
  // Points -> Polygon
  std::cout << "Construct the Extrusion from Concave Hull with Open Cascade..." << std::endl;
  gp_Pnt ppoint;
  TopoDS_Wire wire;
  TopoDS_Face face;
  TopoDS_Shape solid;
  BRepBuilderAPI_MakePolygon mpoly;
  for (size_t j = 0; j < cloud_buffer->points.size() ; ++j)
    {
    ppoint.SetX(cloud_buffer->points[j].x);
    ppoint.SetY(cloud_buffer->points[j].y);
    ppoint.SetZ(cloud_buffer->points[j].z);  
    mpoly.Add(ppoint);
    }
  // Polygon -> Wire -> Face -> Solid
  mpoly.Close();
  wire = mpoly.Wire();
  face = BRepLib_MakeFace (wire);
  solid = BRepPrimAPI_MakePrism(face,gp_Vec(0.,0.,minPt.z-maxPt.z));

  //write Files
  STEPControl_Writer OCC_writer;
  OCC_writer.Transfer(solid,STEPControl_AsIs);
  OCC_writer.Write("E13_Extrusion.stp");
}


