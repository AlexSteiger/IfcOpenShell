#include <string>
#include <iostream>
#include <fstream>
#include <QString>

//OCC:
#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
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

#include <BRepBuilderAPI_MakeSolid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

int main() {

	std::cout << "starting reading the stp-file" << std::endl;
	STEPControl_Reader stpreader;
	stpreader.ReadFile("E8_Spundwand.stp");
	stpreader.TransferRoots();
	TopoDS_Shape spwand_shape = stpreader.OneShape();
  std::cout << "finish reading the stp-file" << std::endl;

	std::cout << "starting reading the stl-file" << std::endl;
  StlAPI_Reader stlreader;
	QString qfilename = "Verformung_cutout.stl";
	TopoDS_Shape verformung_shape;
	stlreader.Read(verformung_shape, (Standard_CString)qfilename.toLatin1().constData());
	std::cout << "finish reading the stl-file" << std::endl;
 	
	
	std::cout << "building shell from faces... (one \"l\" for each face)" << std::endl;
	BRep_Builder builder;
	TopoDS_Shell verformung_shell;
	builder.MakeShell(verformung_shell);
for( TopExp_Explorer ex(verformung_shape, TopAbs_FACE); ex.More(); ex.Next() )
{
	TopoDS_Face currentFace = TopoDS::Face( ex.Current() );
	BRepAdaptor_Surface brepAdaptorSurface( currentFace,Standard_True );
	builder.Add(verformung_shell, currentFace);
	std::cout << "l";
}
	
	TopoDS_Solid solid2 = BRepBuilderAPI_MakeSolid (verformung_shell);
	TopoDS_Shape fused = BRepAlgoAPI_Fuse(spwand_shape,solid2);
	
	//This tool tries to unify faces and edges of the shape which lie on the same geometry
	ShapeUpgrade_UnifySameDomain unif(fused, true, true, false);
	unif.Build();
	TopoDS_Shape fixed = unif.Shape();
	
	//TopoDS_Solid solid3 = TopoDS::Solid(fused);
	// This tries to put the holes in the spwand_shape, but doesn't quite work yet
	// 1. Use BRepPrimAPI_MakeHalfSpace to create a half-space.
	// 2. Use Boolean APIs do sub and intersection operations
	// Waterside: (10,-375,-5) ; Landside: (300,300,-5)
	// Works only for sections of the spwand_shape, which are through the shell "cut off" of the rest
	//TopoDS_Solid Halbraum = BRepPrimAPI_MakeHalfSpace(verformung_shell, gp_Pnt(10,-375,-5)).Solid();
	//TopoDS_Shape Remaining_Extrusion = BRepAlgoAPI_Cut(spwand_shape,Halbraum);
	//TopoDS_Solid Halbraum2 = BRepPrimAPI_MakeHalfSpace(verformung_shell, gp_Pnt(300,300,-5)).Solid();
	//TopoDS_Shape Remaining_Extrusion2 = BRepAlgoAPI_Cut(spwand_shape,Halbraum2);
	

	
	
	//write Files
	STEPControl_Writer OCC_writer;
	OCC_writer.Transfer(fixed,STEPControl_AsIs);
 	OCC_writer.Write("F1.stp");


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


