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

//OCC
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <gp_Pnt.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../../../PCL_Hydromapper/Samples_PCD/Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  pcl::PCDWriter writer;
  writer.write ("1_Input_Cloud.pcd", *cloud, false);  
  
  // Some informations about input Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  std::cout << "Dimensions of Input (";
  std::cout << cloud->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;

  // std::cout << "\nIntervall for filtering: " << std::endl;
  // std::cout << "[ " << maxPt.z-2 << " | " << maxPt.z-1 << " ]" << std::endl;
  
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (maxPt.z-1.5, maxPt.z-1);   //only data points between get through
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);
  writer.write ("2_Cloud_filtered.pcd", *cloud_filtered, false);    // save File
  
  // Some informations about filtered Cloud
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
  std::cout << "\nDimensions of filtered Cloud (";
  std::cout << cloud_filtered->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
 
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  
  // Project
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  writer.write ("3_Cloud_projected.pcd", *cloud_projected, false);    //save File
 
  // Concave Hull
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.05);  //limits the size of the hull semements, smaller -> more Detail
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
  writer.write ("4_Concave_Hull.pcd", *cloud_hull, false);
  
 //////////////////////////////////
//////////OCC STARTS HERE///////////
 //////////////////////////////////
  
  std::cout << "cloud_hull is type: " << typeid(cloud_hull).name() << std::endl;  
  std::cout << "chull is type: " << typeid(chull).name() << std::endl;
//  std::cout << "cloud_hull is type: " << typeid(cloud_hull).name() << std::endl;
//  std::cout << "cloud_hull is type: " << typeid(cloud_hull).name() << std::endl;  
  
  

//   gp_Pnt cloud_hull_P1;
//   gp_Pnt cloud_hull_P2;
  BRepBuilderAPI_MakePolygon mpoly;
  
//   cloud_hull_P1.SetX(cloud_hull->points[0].x);
//   cloud_hull_P1.SetY(cloud_hull->points[0].y);
//   cloud_hull_P1.SetZ(cloud_hull->points[0].z); 
//   
//   cloud_hull_P2.SetX(cloud_hull->points[1].x);
//   cloud_hull_P2.SetY(cloud_hull->points[1].y);
//   cloud_hull_P2.SetZ(cloud_hull->points[1].z);
    
  //for (size_t i = 0; i < cloud_hull->points.size() ; ++i)
  for (size_t i = 0; i < 5 ; ++i)
    {
    
    std::cout << i << ": x : " <<  cloud_hull->points[i].x << std::endl;  
    std::cout << i << ": y : " <<  cloud_hull->points[i].y << std::endl;
    std::cout << i << ": y : " <<  cloud_hull->points[i].y << std::endl; 
  
  
    gp_Pnt ppoint;
    ppoint.SetX(cloud_hull->points[i].x);
    ppoint.SetY(cloud_hull->points[i].y);
    ppoint.SetZ(cloud_hull->points[i].z);  
  
    mpoly.Add(ppoint);
    }
    mpoly.Close();
  
    TopoDS_Wire wire = mpoly.Wire();
    
    STEPControl_Writer OCC_writer;
    OCC_writer.Transfer(wire,STEPControl_AsIs);
    OCC_writer.Write("theCoDe.stp");
  
  
 //////////////////////////////////
//////////IFC STARTS HERE///////////
 //////////////////////////////////
    
  
	// The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
	// convenience functions for working with geometry in IFC files.
	IfcHierarchyHelper file;
	file.header().file_name().name("hellowall.ifc");

	// Start by adding a wall to the file, initially leaving most attributes blank.
	IfcSchema::IfcWallStandardCase* wall = new IfcSchema::IfcWallStandardCase(
		guid(), 			// GlobalId
		0, 					// OwnerHistory
		0,               	// Name
		S("Das ist eine Wand"), 				// Description
		null, 				// ObjectType
		0, 					// ObjectPlacement
		0, 					// Representation
		null				// Tag
#ifdef USE_IFC4
		, IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
#endif
	);

    file.addBuildingProduct(wall);
	// By adding a wall, a hierarchy has been automatically created that consists of the following
	// structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall

	// An IfcOwnerHistory has been initialized as well, which should be assigned to the wall.
	wall->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());

	// The wall will be shaped as a box, with the dimensions specified in millimeters. The resulting
	// product definition will consist of both a body representation as well as an axis representation
	// that runs over the centerline of the box in the X-axis.
	IfcSchema::IfcProductDefinitionShape* south_wall_shape = file.addAxisBox(10000, 360, 3000);

	// Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy
	// of placements for the products
	IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();

	// The shape has to be assigned to the representation of the wall and is placed at the origin
	// of the coordinate system.
	wall->setRepresentation(south_wall_shape);
	wall->setObjectPlacement(file.addLocalPlacement(storey_placement));

	// A pale white colour is assigned to the wall.
	IfcSchema::IfcPresentationStyleAssignment* wall_colour = file.setSurfaceColour(
    south_wall_shape, 0.75, 0.73, 0.68);
    
    std::ofstream f("HelloWall.ifc");
	f << file;

}
