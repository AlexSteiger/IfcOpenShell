/********************************************************************************
 * This Software is available under the the terms of the Lesser GNU General     * 
 * Public License as published by the Free Software Foundation,                 *
 * either version 3.0 of the License, or (at your option) any later version.    *
 ********************************************************************************/                                                                                                                     


#include <string>
#include <iostream>
#include <typeinfo>       // operator typeid
#include <fstream>

#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

#include <Geom_BSplineSurface.hxx>

#include "STEPControl_Writer.hxx"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>

#include <BRepAlgoAPI_Cut.hxx>

#include <Standard_Version.hxx>

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

typedef std::string S;
typedef IfcParse::IfcGlobalId guid;
boost::none_t const null = boost::none;
// The creation of Nurbs-surface for the IfcSite mesh, to be implemented lateron
void createGroundShape(TopoDS_Shape& shape);

int main() {

	// The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
	// convenience functions for working with geometry in IFC files.
	IfcHierarchyHelper file;
	file.header().file_name().name("IfcAdvancedHouse.ifc");

	// To demonstrate the ability to serialize arbitrary opencascade solids a building envelope is
	// constructed by applying boolean operations. Naturally, in IFC, building elements should be 
	// modeled separately, with rich parametric and relational semantics. Creating geometry in this
	// way does not preserve any history and is merely a demonstration of technical capabilities.
	TopoDS_Shape outer =   BRepPrimAPI_MakeBox(gp_Pnt(-9000., -  0., -5000.), gp_Pnt(9000., 2000., 4000.)).Shape();
	TopoDS_Shape inner =   BRepPrimAPI_MakeBox(gp_Pnt(-4640.,  180.,     0.), gp_Pnt(4640., 4820., 3000.)).Shape();
	TopoDS_Shape window1 = BRepPrimAPI_MakeBox(gp_Pnt(-5000., -180.,   400.), gp_Pnt( 500., 1180., 2000.)).Shape();
	TopoDS_Shape window2 = BRepPrimAPI_MakeBox(gp_Pnt( 6000.,    0., -2000.), gp_Pnt(8000., 2000., 2000.)).Shape();
    
	TopoDS_Shape building_shell = BRepAlgoAPI_Cut(
		BRepAlgoAPI_Cut(
			BRepAlgoAPI_Cut(outer, inner),
			window1
			),
		window2
	);
    
    TopoDS_Shape building_shell_v2 = BRepAlgoAPI_Cut(BRepAlgoAPI_Cut(outer, inner),window2);
    TopoDS_Shape building_shell_v3 = BRepAlgoAPI_Cut(outer,window2);

    //First Wall
    IfcSchema::IfcWallStandardCase* wall = new IfcSchema::IfcWallStandardCase(
        guid(),
        0,                      // GlobalId
        0,                      // Owner History
        S("Das ist eine Mauer"), // Description
        null,                   // Object Type
        0,                      // Object Placement
        0,                      // Representation
        null                    // Tag
#ifdef USE IFC4
        , IfcSchema::IfcWallTypeEnum::IfcWallType Standard
#endif
        );
    file.addBuildingProduct(wall);
 	// By adding a building, a hierarchy has been automatically created that consists of the following
	// structure: IfcProject > IfcSite > IfcBuilding                                                                             
    wall->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
    IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();
    wall->setObjectPlacement(file.addLocalPlacement(storey_placement, 0, 20000, 0));
    IfcSchema::IfcProductDefinitionShape* wall_shape = IfcGeom::serialise(outer, false);
	file.addEntity(wall_shape);
	IfcSchema::IfcRepresentation* rep = *wall_shape->Representations()->begin();
	rep->setContextOfItems(file.getRepresentationContext("model"));
	wall->setRepresentation(wall_shape);
    
    //Second Wall
    IfcSchema::IfcWallStandardCase* wall2 = new IfcSchema::IfcWallStandardCase(
        guid(),
        0,                      // GlobalId
        0,                      // Owner History
        S("Das ist eine Mauer"), // Description
        null,                   // Object Type
        file.addLocalPlacement(storey_placement, 0, 10000, 0),                      // Object Placement
        0,                      // Representation
        null                    // Tag
#ifdef USE IFC4
        , IfcSchema::IfcWallTypeEnum::IfcWallType Standard
#endif
        );
    file.addBuildingProduct(wall2);                                                                          
    wall2->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
    IfcSchema::IfcProductDefinitionShape* wall2_2_shape = IfcGeom::serialise(building_shell_v3, false);
	file.addEntity(wall2_2_shape);
    IfcSchema::IfcRepresentation* rep2 = *wall2_2_shape->Representations()->begin();
	rep2->setContextOfItems(file.getRepresentationContext("model"));
	wall2->setRepresentation(wall2_2_shape);
    
    //Second Wall_copy
    IfcSchema::IfcWallStandardCase* wall2_2 = new IfcSchema::IfcWallStandardCase(
        guid(),
        0,                      // GlobalId
        0,                      // Owner History
        S("Das ist eine Mauer"), // Description
        null,                   // Object Type
        file.addLocalPlacement(storey_placement, 0, 0, 0),                      // Object Placement
        0,                      // Representation
        null                    // Tag
#ifdef USE IFC4
        , IfcSchema::IfcWallTypeEnum::IfcWallType Standard
#endif
        );
    file.addBuildingProduct(wall2_2);                                                                          
    wall2_2->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
    IfcSchema::IfcProductDefinitionShape* wall2_shape = IfcGeom::serialise(building_shell_v3, false);
	file.addEntity(wall2_shape);
    IfcSchema::IfcRepresentation* rep2_2 = *wall2_shape->Representations()->begin();
	rep2_2->setContextOfItems(file.getRepresentationContext("model"));
	wall2_2->setRepresentation(wall2_shape);
    
    // Third Wall
    IfcSchema::IfcWallStandardCase* wall3 = new IfcSchema::IfcWallStandardCase(
        guid(),
        0,                      // GlobalId
        0,                      // Owner History
        S("Das ist eine Mauer"), // Description
        null,                   // Object Type
        file.addLocalPlacement(storey_placement, 0, -10000, 0),                      // Object Placement
        0,                      // Representation
        null                    // Tag
#ifdef USE IFC4
        , IfcSchema::IfcWallTypeEnum::IfcWallType Standard
#endif
        );
    file.addBuildingProduct(wall3);
 	// By adding a building, a hierarchy has been automatically created that consists of the following
	// structure: IfcProject > IfcSite > IfcBuilding                                                                             
    wall3->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());
    
	// For the ground mesh of the IfcSite we will use a Nurbs surface created in Open Cascade. Only
	// in IFC4 the surface can be directly serialized. In IFC2X3 the it will have to be tesselated.
	TopoDS_Shape shape;
	createGroundShape(shape);
    
    //Trying to make a Triangula Mesh
    double const deflection = 1000;
    double const angulardeflection = 1000;
    BRepMesh_IncrementalMesh tess(shape, deflection, angulardeflection);
    tess.Perform();
    TopoDS_Face meshFace = TopoDS::Face(tess.Shape());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(meshFace, loc);
    //Cut a solid with a face
    // 1. Use BRepPrimAPI_MakeHalfSpace to create a half-space.    
    TopoDS_Solid Halbraum = BRepPrimAPI_MakeHalfSpace(meshFace, gp_Pnt( 0., 1180., 5000.)).Solid();
    // 2. Use Boolean APIs do sub and intersection operations
    TopoDS_Shape UpperHalf = BRepAlgoAPI_Cut(building_shell_v3,Halbraum);

    // Since the solid consists only of planar faces and straight edges it can be serialized as an
	// IfcFacetedBRep. If it would not be a polyhedron, serialise() can only be successful when linked
	// to the IFC4 model and with `advanced` set to `true` which introduces IfcAdvancedFace. It would
	// return `0` otherwise.
    IfcSchema::IfcProductDefinitionShape* wall3_shape = IfcGeom::serialise(shape, true);    
    wall3_shape = IfcGeom::tesselate(UpperHalf, 100.);
 	file.addEntity(wall3_shape);
 	IfcSchema::IfcRepresentation* represent = *wall3_shape->Representations()->begin();   
    represent->setContextOfItems(file.getRepresentationContext("model"));
	wall3->setRepresentation(wall3_shape);
	// A pale white colour is assigned to the building.
	file.setSurfaceColour(wall3_shape, 0.75, 0.73, 0.68);

	// Finally create a file stream for our output and write the IFC file to it.
	std::ofstream f("Mauer.ifc");
	f << file;
    
    STEPControl_Writer writer;
    //writer.Transfer(outer,STEPControl_AsIs);
    writer.Transfer(building_shell_v3,STEPControl_AsIs);
    writer.Transfer(shape,STEPControl_AsIs);
    //writer.Transfer(UpperHalf,STEPControl_AsIs);    
    writer.Write("Mauer.stp");
    
    std::cout << "building_shell is type: " << typeid(building_shell).name() << std::endl;
    std::cout << "shape is type: " << typeid(shape).name() << std::endl;  
    std::cout << "tess is type: " << typeid(tess).name() << std::endl; 
    std::cout << "meshFace is type: " << typeid(meshFace).name() << std::endl; 
    std::cout << "triangulation is type: " << typeid(triangulation).name() << std::endl;     
    std::cout << "Halbraum is type: " << typeid(Halbraum).name() << std::endl;  
}

void createGroundShape(TopoDS_Shape& shape) {
	TColgp_Array2OfPnt cv (0, 4, 0, 4);
	cv.SetValue(0, 0, gp_Pnt(-10000,  -5000, -4130));
	cv.SetValue(0, 1, gp_Pnt(-10000,  -2330, -1130));
	cv.SetValue(0, 2, gp_Pnt(-10000,      0,  -530));
	cv.SetValue(0, 3, gp_Pnt(-10000,   2330,  -130));
    cv.SetValue(0, 4, gp_Pnt(-10000,   5000,  2000));
	cv.SetValue(1, 0, gp_Pnt( -3330,  -5000, -5130));
	cv.SetValue(1, 1, gp_Pnt( -7670,  -2670,  2000));
	cv.SetValue(1, 2, gp_Pnt( -9000,      0,  1000));
	cv.SetValue(1, 3, gp_Pnt( -7670,   4670,  3000));
    
	cv.SetValue(1, 4, gp_Pnt( -3330,   5000,  4130));
	cv.SetValue(2, 0, gp_Pnt(     0,  -5000, -5530));
	cv.SetValue(2, 1, gp_Pnt(     0,  -2670,  3000));
	cv.SetValue(2, 2, gp_Pnt(     0,      0, -9000));
	cv.SetValue(2, 3, gp_Pnt(     0,   4670,  1500));
	cv.SetValue(2, 4, gp_Pnt(     0,   5000,  4130));
	cv.SetValue(3, 0, gp_Pnt(  3330,  -5000, -6130));
	cv.SetValue(3, 1, gp_Pnt(  7670,  -1670,  6000));
	cv.SetValue(3, 2, gp_Pnt(  9000,      0,  5000));
	cv.SetValue(3, 3, gp_Pnt(  7670,   4500,  7000));
	cv.SetValue(3, 4, gp_Pnt(  3330,   5000,  4130));
	cv.SetValue(4, 0, gp_Pnt( 10000,  -5000, -6130));
	cv.SetValue(4, 1, gp_Pnt( 10000,  -2330,  5130));
	cv.SetValue(4, 2, gp_Pnt( 10000,      0,  4130));
	cv.SetValue(4, 3, gp_Pnt( 10000,   2330,  4130));
	cv.SetValue(4, 4, gp_Pnt( 10000,   5000,  8130));
	TColStd_Array1OfReal knots(0, 1);
	knots(0) = 0;
	knots(1) = 1;		
	TColStd_Array1OfInteger mult(0, 1);
	mult(0) = 5;
	mult(1) = 5;	
	Handle(Geom_BSplineSurface) surf = new Geom_BSplineSurface(cv, knots, knots, mult, mult, 4, 4);
#if OCC_VERSION_HEX < 0x60502
	shape = BRepBuilderAPI_MakeFace(surf);
#else
	shape = BRepBuilderAPI_MakeFace(surf, Precision::Confusion());
#endif
}
