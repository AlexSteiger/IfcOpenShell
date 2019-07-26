#include <string>
#include <iostream>
#include <fstream>
#include <QString>
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

	//The Clouds: input, semgmentation, projection, concave
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Input Cloud
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("Spundwand.pcd", cloud_blob);
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
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_input);
  sor2.setMeanK (100);
  sor2.setStddevMulThresh (5);
  sor2.setNegative (false);
  sor2.filter (*cloud_filtered1);
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
  seg.setFilterLimits (maxPt.z - 2, maxPt.z);   //(unten, oben)
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
  
  // Statistical outlier removal
  std::cout << "Starting Statistical outlier removal of projected cloud... " ;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_projected);
  sor.setMeanK (20);
  sor.setStddevMulThresh (2);
  sor.setNegative (false);
  sor.filter (*cloud_filtered);
  std::cout << cloud_projected->points.size() - cloud_filtered->points.size ();
  std::cout << " outliers where removed. ";
  std::cout << cloud_filtered->points.size() << " points left" << std::endl;

  // Concave Hull
  std::cout << "Construct the Concave Hull... ";
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_filtered);
  chull.setAlpha (0.06);  //limits the size of the hull semements, smaller->more Detail
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

	std::cout << "starting reading the stl-file" << std::endl;
  StlAPI_Reader stlreader;
	QString qfilename = "Verformung_cutout.stl";
	TopoDS_Shape verformung_shape;
	stlreader.Read(verformung_shape, (Standard_CString)qfilename.toLatin1().constData());
	std::cout << "finish reading the stl-file" << std::endl;
 	
	
	std::cout << "building shell from faces... (one \"I\" for each face)" << std::endl;
	BRep_Builder builder;
	TopoDS_Shell verformung_shell;
	builder.MakeShell(verformung_shell);
for( TopExp_Explorer ex(verformung_shape, TopAbs_FACE); ex.More(); ex.Next() )
{
	TopoDS_Face currentFace = TopoDS::Face( ex.Current() );
	BRepAdaptor_Surface brepAdaptorSurface( currentFace,Standard_True );
	builder.Add(verformung_shell, currentFace);
	//std::cout << "I";
}
	
	// Cut a solid with a face
	// 1. Use BRepPrimAPI_MakeHalfSpace to create a half-space. 
	// One side: 150.,0.,-5.
	// Other side: 
	TopoDS_Solid Halbraum = BRepPrimAPI_MakeHalfSpace(verformung_shell, gp_Pnt(150.,-100.,-5.)).Solid();
	// 2. Use Boolean APIs do sub and intersection operations
 	TopoDS_Shape Remaining_Extrusion = BRepAlgoAPI_Cut(solid,Halbraum);

  //write Files
  pcl::PCDWriter writer;
  writer.write ("E1_0_Input_Cloud.pcd", *cloud_input, false);
  writer.write ("E2_Cloud_filtered.pcd", *cloud_filtered1);
  writer.write ("E3_cloud_smoothed.pcd", *cloud_smoothed);
  writer.write ("E4_Cloud_segmented.pcd", *cloud_segmented, false);      
  writer.write ("E5_Cloud_projected.pcd", *cloud_projected, false);    
  writer.write ("E6_Cloud_inliers.pcd", *cloud_filtered, false);
  writer.write ("E7_Concave_Hull.pcd", *cloud_hull, false);
  STEPControl_Writer OCC_writer;
  OCC_writer.Transfer(solid,STEPControl_AsIs);
	//OCC_writer.Transfer(verformung_shell,STEPControl_AsIs);
	//OCC_writer.Write("E9_Remaining_Extrusion.stp");
  OCC_writer.Write("E8_Spundwand.stp");


//////////////////////////////////
//////////IFC STARTS HERE/////////
//////////////////////////////////
    
  
//   // The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
//   // convenience functions for working with geometry in IFC files.
//   IfcHierarchyHelper file;
//   file.header().file_name().name("E8_Spundwand.ifc");
// 
//   // Start by adding a wall to the file, initially leaving most attributes blank.
//   IfcSchema::IfcWallStandardCase* wall = new IfcSchema::IfcWallStandardCase(
//     guid(), 		// GlobalId
//     0, 					// OwnerHistory
//     S("Spundwand"),     // Name
//     S("Automatisch erstellt durch extrusion.cpp"), 	// Description
//     null, 			// ObjectType
//     0, 					// ObjectPlacement
//     0, 					// Representation
//     null				// Tag
//   #ifdef USE_IFC4
//     , IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
//   #endif
//   );
// 
//   file.addBuildingProduct(wall);
//   // By adding a wall, a hierarchy has been automatically created that consists of the following
//   // structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
// 
//   // An IfcOwnerHistory has been initialized as well, which should be assigned to the wall.
//   wall->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
// 
//   // Since the solid consists only of planar faces and straight edges it can be serialized as an
//   // IfcFacetedBRep. If it would not be a polyhedron, serialise() can only be successful when linked
//   // to the IFC4 model and with `advanced` set to `true` which introduces IfcAdvancedFace. It would
//   // return `0` otherwise.
//   IfcSchema::IfcProductDefinitionShape* object_shape = IfcGeom::serialise(solid, false);
//   file.addEntity(object_shape);
//   IfcSchema::IfcRepresentation* rep = *object_shape->Representations()->begin();
//   rep->setContextOfItems(file.getRepresentationContext("model"));
//   wall->setRepresentation(object_shape);
//   // A red colour is assigned to the wall.
//   file.setSurfaceColour(object_shape, 0.55 , 0.25 , 0.05);
//     
//   // Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy of placements for the products
//   IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();
// 
//   // wall and is placed at the origin of the coordinate system.
//   wall->setObjectPlacement(file.addLocalPlacement(storey_placement));
// 
// 
// 
//   std::ofstream f("E8_Spundwand.ifc");
//   f << file;

}


