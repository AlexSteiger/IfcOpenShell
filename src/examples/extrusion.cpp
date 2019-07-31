#include <string>
#include <iostream>
#include <fstream>
#include <QString>
//PCL:
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //for getMinMax3D
#include <pcl/ModelCoefficients.h>  //for planar coefficients
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/surface/mls.h>    //MovingLeastSquares
//OCC:
#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS.hxx>
#include "STEPControl_Writer.hxx"

int main(int argc, char *argv[]) {

	//The Clouds: input, semgmentation, projection, concave
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  
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
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
  // Some informations about input Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_input, minPt, maxPt); 
  std::cout << "Dimensions of Input (";
  std::cout << cloud_input->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;

  // Statistical outlier removal 1
  std::cout << "Starting Statistical outlier removal... ";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
  sor1.setInputCloud (cloud_input);
  sor1.setMeanK (100);
  sor1.setStddevMulThresh (5);
  sor1.setNegative (false);
  sor1.filter (*cloud_filtered1);
  std::cout << cloud_input->points.size() - cloud_filtered1->points.size () << " outliers where removed. ";
  std::cout << cloud_filtered1->points.size() << " points left" << std::endl;
  
  // MovingLeastSquares 
  std::cout << "Starting Moving Least Squares... ";
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //(<Input_Cloud, Output_Cloud>)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
  mls.setInputCloud (cloud_filtered1);
  mls.setComputeNormals (false);
  mls.setSearchRadius (0.1);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder (3);
  mls.process (*cloud_smoothed);
  std::cout << cloud_smoothed->points.size () << " points left." << std::endl;
  
  // Segment the cloud
  std::cout << "Starting segmenting the Cloud... " << std::endl;
  pcl::PassThrough<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud_smoothed);
  seg.setFilterFieldName ("z");
  seg.setFilterLimits (maxPt.z - 3, maxPt.z);   //(unten, oben)
  seg.setFilterLimitsNegative (false);
  seg.filter (*cloud_segmented);
  // Some informations about the segmented Cloud
  pcl::PointXYZ minPt_f, maxPt_f;
  pcl::getMinMax3D (*cloud_segmented, minPt_f, maxPt_f);
  std::cout << "Dimensions of the segmented Cloud (";
  std::cout << cloud_segmented->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt_f.x << " | Max x: " << maxPt_f.x << std::endl;
  std::cout << "Min y: " << minPt_f.y << " | Max y: " << maxPt_f.y << std::endl;
  std::cout << "Min z: " << minPt_f.z << " | Max z: " << maxPt_f.z << std::endl;

  // Create a set of planar coefficients (The plane coefficients are: a, b, c, d (ax+by+cz+d=0)) 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;  // x = y = 0
  coefficients->values[2] = 1;                            // z 
  coefficients->values[3] = -maxPt.z;                   // d (Hessesche Normalform)
  
  // Project
  std::cout << "Starting projecting the Cloud... " << std::endl;
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_segmented);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  // Some informations about projected Cloud
  pcl::PointXYZ minPt_p, maxPt_p;
  pcl::getMinMax3D (*cloud_projected, minPt_p, maxPt_p);
  std::cout << "Dimensions of projected Cloud (";
  std::cout << cloud_projected->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt_p.x << " | Max x: " << maxPt_p.x << std::endl;
  std::cout << "Min y: " << minPt_p.y << " | Max y: " << maxPt_p.y << std::endl;
  std::cout << "Min z: " << minPt_p.z << " | Max z: " << maxPt_p.z << std::endl;
  
  // Statistical outlier removal 2
  std::cout << "Starting Statistical outlier removal of projected cloud... " ;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_projected);
  sor2.setMeanK (40);
  sor2.setStddevMulThresh (2.5);
  sor2.setNegative (false);
  sor2.filter (*cloud_filtered);
  std::cout << cloud_projected->points.size() - cloud_filtered->points.size ();
  std::cout << " outliers where removed. ";
  std::cout << cloud_filtered->points.size() << " points left" << std::endl;

  // Concave Hull
  std::cout << "Construct the Concave Hull... ";
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_filtered);
  chull.setAlpha (0.07);  //limits the size of the hull semements, smaller->more Detail
  chull.reconstruct (*cloud_hull);
  std::cerr << "Concave Hull has " << cloud_hull->points.size () << " data points." << std::endl;

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
  for (size_t j = 0; j < cloud_hull->points.size() ; ++j)
    {
    ppoint.SetX(cloud_hull->points[j].x);
    ppoint.SetY(cloud_hull->points[j].y);
    ppoint.SetZ(cloud_hull->points[j].z);  
    mpoly.Add(ppoint);
    }
  // Polygon -> Wire -> Face -> Solid
  mpoly.Close();
  wire = mpoly.Wire();
  face = BRepLib_MakeFace (wire);
  solid = BRepPrimAPI_MakePrism(face,gp_Vec(0.,0.,minPt.z-maxPt.z));

  //write Files
  pcl::PCDWriter writer;
  writer.write ("E0_Input_Cloud.pcd", *cloud_input, false);
  writer.write ("E1_Cloud_filtered.pcd", *cloud_filtered1);
  writer.write ("E2_cloud_smoothed.pcd", *cloud_smoothed);
  writer.write ("E3_Cloud_segmented.pcd", *cloud_segmented, false);      
  writer.write ("E4_Cloud_projected.pcd", *cloud_projected, false);    
  writer.write ("E5_Cloud_inliers.pcd", *cloud_filtered, false);
  writer.write ("E6_Concave_Hull.pcd", *cloud_hull, false);
  STEPControl_Writer OCC_writer;
  OCC_writer.Transfer(solid,STEPControl_AsIs);
  OCC_writer.Write("E7_Extrusion.stp");
}


