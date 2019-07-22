//PCL
#include <pcl/io/pcd_io.h>  //for PointCloud
#include <pcl/point_types.h>
#include <pcl/common/common.h> //for getMinMax3D
#include <pcl/ModelCoefficients.h>  //for planar coefficients
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/reconstruction.h>

#include <string>
#include <iostream>
#include <fstream>
#include <typeinfo>       // operator typeid
#include <boost/format.hpp>

//OCC
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <gp_Pnt.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <Standard_Version.hxx>
#include <TopoDS_Wire.hxx>
#include "STEPControl_Writer.hxx"

//IFC
#ifdef USE_IFC4
#include "../ifcparse/Ifc4.h"
#else
#include "../ifcparse/Ifc2x3.h"
#endif

#include "../ifcparse/IfcBaseClass.h"
#include "../ifcparse/IfcHierarchyHelper.h"
#include "../ifcgeom/IfcGeom.h"

#if USE_VLD
#include <vld.h>
#endif

// Some convenience typedefs and definitions. 
typedef std::string S;
typedef IfcParse::IfcGlobalId guid;
typedef std::pair<double, double> XY;
boost::none_t const null = boost::none;

int main() {

  //input
  pcl::PCLPointCloud2 cloud_blob;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  
  //semgmentation, projection, concave
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;

  pcl::PointXYZ minPt, maxPt;
  pcl::PointXYZ minPt_f, maxPt_f;
  pcl::PointXYZ minPt_p, maxPt_p; 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  
  //writer
  pcl::PCDWriter writer;
  STEPControl_Writer OCC_writer;
  
  //OpenCascade
  gp_Pnt ppoint;
  TopoDS_Wire wire;
  TopoDS_Face face;
  BRepOffsetAPI_ThruSections thruSec (1,0, 1.0e-03);
  
  // Input Cloud
  pcl::io::loadPCDFile ("../../../PCL_Hydromapper/Samples_PCD/Spundwand_clean.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
  writer.write ("1_Input_Cloud.pcd", *cloud_input, false);  
  
  // Some informations about input Cloud
  pcl::getMinMax3D (*cloud_input, minPt, maxPt); 
  std::cout << "Dimensions of Input (";
  std::cout << cloud_input->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
  
  double unten;
  double oben;
  double segHeight;
  for (int i = 0; i <= 2; i++)
    {
    switch (i)
    {
      case 0:
        {
        unten = maxPt.z - 0.4;              
        oben = maxPt.z;
        segHeight = oben;
        break;
        }
      case 1:
        {
        unten=(minPt.z+maxPt.z)*0.5-0.2;  
        oben = (minPt.z+maxPt.z)*0.5+0.2;
        segHeight = (unten+oben)*0.5;
        break;
        }
      case 2:
      {
      unten = minPt.z;              
      oben = minPt.z + 0.4; 
      segHeight = unten; 
      break;
      }
    }
    //std::cout << "[ " << unten << " | " << oben << " ]" << std::endl;
    
    // Segment the cloud
    pass.setInputCloud (cloud_input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (unten, oben);   //(unten, oben)
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);
    writer.write ("2_Cloud_filtered.pcd", *cloud_filtered, false);    // save File
    
    // Some informations about the segmented Cloud
    pcl::getMinMax3D (*cloud_filtered, minPt_f, maxPt_f);
    std::cout << "\nDimensions of the segmented Cloud (";
    std::cout << cloud_filtered->points.size () << " data points):" << std::endl;
    std::cout << "Min x: " << minPt_f.x << " | Max x: " << maxPt_f.x << std::endl;
    std::cout << "Min y: " << minPt_f.y << " | Max y: " << maxPt_f.y << std::endl;
    std::cout << "Min z: " << minPt_f.z << " | Max z: " << maxPt_f.z << std::endl;
  
    // Create a set of planar coefficients (The plane coefficients are: a, b, c, d (ax+by+cz+d=0)) 
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;  // x = y = 0
    coefficients->values[2] = 1;                            // z 
    coefficients->values[3] = -segHeight;                     // d (Hessesche Normalform)
    
    // Project
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    writer.write ("3_Cloud_projected.pcd", *cloud_projected, false);    //save File
  
    // Some informations about projected Cloud
    pcl::getMinMax3D (*cloud_projected, minPt_p, maxPt_p);
    std::cout << "\nDimensions of projected Cloud (";
    std::cout << cloud_projected->points.size () << " data points):" << std::endl;
    std::cout << "Min x: " << minPt_p.x << " | Max x: " << maxPt_p.x << std::endl;
    std::cout << "Min y: " << minPt_p.y << " | Max y: " << maxPt_p.y << std::endl;
    std::cout << "Min z: " << minPt_p.z << " | Max z: " << maxPt_p.z << std::endl;
    
    // Concave Hull
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.1);  //limits the size of the hull semements, smaller -> more Detail
    chull.reconstruct (*cloud_hull);

    std::string Concave_string = str(boost::format("4_Concave_Hull%1%.pcd") % i);
    char *nstring2 = new char[32];
    strcpy(nstring2, Concave_string.c_str());
    
    std::cerr << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    writer.write (nstring2, *cloud_hull, false);

    //Make a Polygon from the concave_hull points
    BRepBuilderAPI_MakePolygon mpoly;
    for (size_t j = 0; j < cloud_hull->points.size() ; ++j)
      {
      // std::cout << j << ": x : " <<  cloud_hull->points[j].x << std::endl;  
      // std::cout << j << ": y : " <<  cloud_hull->points[j].y << std::endl;
      // std::cout << j << ": y : " <<  cloud_hull->points[j].y << std::endl; 

      ppoint.SetX(cloud_hull->points[j].x);
      ppoint.SetY(cloud_hull->points[j].y);
      ppoint.SetZ(cloud_hull->points[j].z);  
      mpoly.Add(ppoint);
      }
    mpoly.Close();
    wire = mpoly.Wire();
    BRepLib_MakeFace mface (wire);
    thruSec.AddWire(wire);
    
//     std::string orig = str(boost::format("Hydromapper%1%.stp") % i);
//     char *nstring = new char[32];
//     strcpy(nstring, orig.c_str());
//     cout << nstring << " (char *)" << endl;
    
    //OCC_writer.Transfer(mface,STEPControl_AsIs);
    //OCC_writer.Transfer(mpoly,STEPControl_AsIs);

  }
  thruSec.CheckCompatibility();
  TopoDS_Shape myshape = thruSec.Shape();
  //thruSec.Build();
  OCC_writer.Transfer(myshape,STEPControl_AsIs);
  OCC_writer.Write("Hydromapper.stp");

  
  
//////////////////////////////////
//////////IFC STARTS HERE/////////
//////////////////////////////////
    
  
// 	// The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
// 	// convenience functions for working with geometry in IFC files.
// 	IfcHierarchyHelper file;
// 	file.header().file_name().name("hellowall.ifc");
// 
// 	// Start by adding a wall to the file, initially leaving most attributes blank.
// 	IfcSchema::IfcWallStandardCase* wall = new IfcSchema::IfcWallStandardCase(
// 		guid(), 			// GlobalId
// 		0, 					// OwnerHistory
// 		0,               	// Name
// 		S("Das ist eine Wand"), 				// Description
// 		null, 				// ObjectType
// 		0, 					// ObjectPlacement
// 		0, 					// Representation
// 		null				// Tag
// #ifdef USE_IFC4
// 		, IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
// #endif
// 	);
// 
//     file.addBuildingProduct(wall);
// 	// By adding a wall, a hierarchy has been automatically created that consists of the following
// 	// structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
// 
// 	// An IfcOwnerHistory has been initialized as well, which should be assigned to the wall.
// 	wall->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
// 
// 	// The wall will be shaped as a box, with the dimensions specified in millimeters. The resulting
// 	// product definition will consist of both a body representation as well as an axis representation
// 	// that runs over the centerline of the box in the X-axis.
// 	IfcSchema::IfcProductDefinitionShape* south_wall_shape = file.addAxisBox(10000, 360, 3000);
// 
// 	// Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy
// 	// of placements for the products
// 	IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();
// 
// 	// The shape has to be assigned to the representation of the wall and is placed at the origin
// 	// of the coordinate system.
// 	wall->setRepresentation(south_wall_shape);
// 	wall->setObjectPlacement(file.addLocalPlacement(storey_placement));
// 
// 	// A pale white colour is assigned to the wall.
// 	IfcSchema::IfcPresentationStyleAssignment* wall_colour = file.setSurfaceColour(
//     south_wall_shape, 0.75, 0.73, 0.68);
//
//    //std::ofstream f("Hydromapper.ifc");
//	//f << file;

}
