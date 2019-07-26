#include <string>
#include <iostream>
#include <fstream>
#include <QString>
#include <vector>
//PCL:
#include <pcl/io/pcd_io.h>  //for PointCloud
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
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include "STEPControl_Writer.hxx"
#include "STEPControl_Reader.hxx"
#include "StlAPI_Reader.hxx"
#include <TopExp_Explorer.hxx>
#include <Standard_PrimitiveTypes.hxx>
#include <BRepAdaptor_Surface.hxx>



// Some convenience typedefs and definitions. 
// typedef std::string S;
// typedef IfcParse::IfcGlobalId guid;
// boost::none_t const null = boost::none;

int main() {

	//The Clouds: input, semgmentation, projection, concave
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);

  
  // Input Cloud
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("E5_Cloud_projected.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
  // Some informations about input Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_input, minPt, maxPt); 
  std::cout << "Dimensions of Input (";
  std::cout << cloud_input->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;

	gp_Pnt pt;
	  for (size_t j = 0; j < cloud_input->points.size () ; ++j)
    {
    pt.SetX(cloud_input->points[j].x);
    pt.SetY(cloud_input->points[j].y);
    pt.SetZ(cloud_input->points[j].z);  
    }
	
	
	
	
	
	  //write Files
//   pcl::PCDWriter writer;
//   writer.write ("M1_0_Input_Cloud.pcd", *cloud_input, false);
//   writer.write ("M2_Cloud_filtered.pcd", *cloud_filtered1);
//   writer.write ("M3_cloud_smoothed.pcd", *cloud_smoothed);
//   writer.write ("M4_Cloud_segmented.pcd", *cloud_segmented, false);      
//   writer.write ("M5_Cloud_projected.pcd", *cloud_projected, false);    
//   writer.write ("M6_Cloud_inliers.pcd", *cloud_filtered, false);
//   writer.write ("M7_Concave_Hull.pcd", *cloud_hull, false);
// 	writer.write ("M8_Ransac.pcd", *middleline, false);
	
	
	
	
	
	
	
	
	
	
	
//   //OpenCascade
//   // Take the the concave_hull points and make a OCC Polygon with it. 
//   // The concave_hull points are ordered clockwise, 
//   // so there is no problem constructing the Polygon.
//   // Points -> Polygon
//   std::cout << "Construct the Extrusion from Concave Hull with Open Cascade..." << std::endl;
//   gp_Pnt ppoint;
//   TopoDS_Wire wire;
//   TopoDS_Face face;
//   TopoDS_Shape solid;
//   BRepBuilderAPI_MakePolygon mpoly;
//   for (size_t j = 0; j < cloud_hull->points.size() ; ++j)
//     {
//     ppoint.SetX(cloud_hull->points[j].x);
//     ppoint.SetY(cloud_hull->points[j].y);
//     ppoint.SetZ(cloud_hull->points[j].z);  
//     mpoly.Add(ppoint);
//     }
//   // Polygon -> Wire -> Face -> Solid
//   mpoly.Close();
//   wire = mpoly.Wire();
//   face = BRepLib_MakeFace (wire);
//   solid = BRepPrimAPI_MakePrism(face,gp_Vec(0.,0.,minPt.z-maxPt.z));

}


