#include <string>
#include <iostream>
#include <fstream>
//PCL:
#include <pcl/io/pcd_io.h>  //for PointCloud
#include <pcl/point_types.h>
#include <pcl/common/common.h> //for getMinMax3D
#include <pcl/ModelCoefficients.h>  //for planar coefficients
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/reconstruction.h>
//OCC:
#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS_Wire.hxx>
#include "STEPControl_Reader.hxx"
#include "STEPControl_Writer.hxx"
//IFC:
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
boost::none_t const null = boost::none;

int main() {

//   //The Clouds:input, semgmentation, projection, concave
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
//   

//   
//   // Input Cloud
//   pcl::PCLPointCloud2 cloud_blob;
//   pcl::io::loadPCDFile ("../../../PCL_Hydromapper/Samples_PCD/Spundwand_easy.pcd", cloud_blob);
//   pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
//   
//   // Some informations about input Cloud
//   pcl::PointXYZ minPt, maxPt;
//   pcl::getMinMax3D (*cloud_input, minPt, maxPt); 
//   std::cout << "Dimensions of Input (";
//   std::cout << cloud_input->points.size () << " data points):" << std::endl;
//   std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
//   std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
//   std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
// 
//   // Segment the cloud
//   pcl::PassThrough<pcl::PointXYZ> seg;
//   seg.setInputCloud (cloud_input);
//   seg.setFilterFieldName ("z");
//   seg.setFilterLimits (maxPt.z - 3, maxPt.z);   //(unten, oben)
//   seg.setFilterLimitsNegative (false);
//   seg.filter (*cloud_segmented);
// 
//   // Some informations about the segmented Cloud
//   pcl::PointXYZ minPt_f, maxPt_f;
//   pcl::getMinMax3D (*cloud_segmented, minPt_f, maxPt_f);
//   std::cout << "\nDimensions of the segmented Cloud (";
//   std::cout << cloud_segmented->points.size () << " data points):" << std::endl;
//   std::cout << "Min x: " << minPt_f.x << " | Max x: " << maxPt_f.x << std::endl;
//   std::cout << "Min y: " << minPt_f.y << " | Max y: " << maxPt_f.y << std::endl;
//   std::cout << "Min z: " << minPt_f.z << " | Max z: " << maxPt_f.z << std::endl;
// 
//   // Create a set of planar coefficients (The plane coefficients are: a, b, c, d (ax+by+cz+d=0)) 
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//   coefficients->values.resize (4);
//   coefficients->values[0] = coefficients->values[1] = 0;  // x = y = 0
//   coefficients->values[2] = 1;                            // z 
//   coefficients->values[3] = -maxPt.z;                   // d (Hessesche Normalform)
//   
//   // Project
//   pcl::ProjectInliers<pcl::PointXYZ> proj;
//   proj.setModelType (pcl::SACMODEL_PLANE);
//   proj.setInputCloud (cloud_segmented);
//   proj.setModelCoefficients (coefficients);
//   proj.filter (*cloud_projected);
// 
//   // Some informations about projected Cloud
//   pcl::PointXYZ minPt_p, maxPt_p;
//   pcl::getMinMax3D (*cloud_projected, minPt_p, maxPt_p);
//   std::cout << "\nDimensions of projected Cloud (";
//   std::cout << cloud_projected->points.size () << " data points):" << std::endl;
//   std::cout << "Min x: " << minPt_p.x << " | Max x: " << maxPt_p.x << std::endl;
//   std::cout << "Min y: " << minPt_p.y << " | Max y: " << maxPt_p.y << std::endl;
//   std::cout << "Min z: " << minPt_p.z << " | Max z: " << maxPt_p.z << std::endl;
//   
//   // Statistical outlier removal
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//   sor.setInputCloud (cloud_projected);
//   sor.setMeanK (60);
//   sor.setStddevMulThresh (3.5);
//   sor.setNegative (false);
//   sor.filter (*cloud_filtered);
//   std::cout << cloud_projected->points.size() - cloud_filtered->points.size () << " outliers where removed. ";
//   std::cout << cloud_filtered->points.size() << " points left" << std::endl;
// 
//   // Concave Hull
//   pcl::ConcaveHull<pcl::PointXYZ> chull;
//   chull.setInputCloud (cloud_filtered);
//   chull.setAlpha (0.05);  //limits the size of the hull semements, smaller -> more Detail
//   chull.reconstruct (*cloud_hull);
//   std::cerr << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
// 
//   // Take the the concave_hull points and make a OCC Polygon with it. 
//   // The concave_hull points are ordered clockwise, 
//   // so there is no problem constructing the Polygon.
//   // Points -> Polygon
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
//   // To 
//   // Polygon -> Wire -> Face -> Solid
//   mpoly.Close();
//   wire = mpoly.Wire();
//   face = BRepLib_MakeFace (wire);
//   solid = BRepPrimAPI_MakePrism(face,gp_Vec(0.,0.,minPt.z-maxPt.z));
//   
//   //write Files
//   pcl::PCDWriter writer;
//   writer.write ("1_Input_Cloud.pcd", *cloud_input, false);  
//   writer.write ("2_Cloud_segmented.pcd", *cloud_segmented, false);      
//   writer.write ("3_Cloud_projected.pcd", *cloud_projected, false);    
//   writer.write ("4_Cloud_inliers.pcd", *cloud_filtered, false);
//   writer.write ("5_Concave_Hull.pcd", *cloud_hull, false);
//   STEPControl_Writer OCC_writer;
//   OCC_writer.Transfer(solid,STEPControl_AsIs);
//   OCC_writer.Write("Hydromapper_v2.stp");
  
  
  STEPControl_Reader OCC_Reader;
  OCC_Reader.ReadFile("Verformungen.stl");
  
  //Standard_Integer NbRoots = reader.NbRootsForTransfer();
  
  TopoDS_Shape solid = OCC_Reader.Shape();
  
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
// 		S("Spundwand"), 				// Description
// 		null, 				// ObjectType
// 		0, 					// ObjectPlacement
// 		0, 					// Representation
// 		null				// Tag
// #ifdef USE_IFC4
// 		, IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
// #endif
// 	);
// 
//   file.addBuildingProduct(wall);
// 	// By adding a wall, a hierarchy has been automatically created that consists of the following
// 	// structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
// 
// 	// An IfcOwnerHistory has been initialized as well, which should be assigned to the wall.
// 	wall->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
// 
// 	// The wall will be shaped as a box, with the dimensions specified in millimeters. The resulting
// 	// product definition will consist of both a body representation as well as an axis representation
// 	// that runs over the centerline of the box in the X-axis.
// 	IfcSchema::IfcProductDefinitionShape* object_shape = IfcGeom::serialise(solid, false);
//   file.addEntity(object_shape);
//   IfcSchema::IfcRepresentation* rep = *object_shape->Representations()->begin();
//   rep->setContextOfItems(file.getRepresentationContext("model"));
//   wall->setRepresentation(object_shape);
//   
// 	// Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy
// 	// of placements for the products
// 	IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();
// 
// 	// wall and is placed at the origin of the coordinate system.
// 	wall->setObjectPlacement(file.addLocalPlacement(storey_placement));
// 
// 	// A pale white colour is assigned to the wall.
// // 	IfcSchema::IfcPresentationStyleAssignment* wall_colour = file.setSurfaceColour(
// //     south_wall_shape, 0.75, 0.73, 0.68);
// 
//   std::ofstream f("Hydromapper.ifc");
// 	f << file;

}