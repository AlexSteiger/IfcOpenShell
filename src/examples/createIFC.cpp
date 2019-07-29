#include <iostream>
//OCC:

#include "STEPControl_Reader.hxx"
#include <TopoDS_Shape.hxx>
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

	std::cout << "starting reading the stp-file" << std::endl;
	STEPControl_Reader stpreader;
	stpreader.ReadFile("F1.stp");
	stpreader.TransferRoots();
	TopoDS_Shape spwand_shape = stpreader.OneShape();
  std::cout << "finish reading the stp-file" << std::endl;

//////////////////////////////////
//////////IFC STARTS HERE/////////
//////////////////////////////////
    
  // The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
  // convenience functions for working with geometry in IFC files.
  IfcHierarchyHelper file;
  file.header().file_name().name("E8_Spundwand.ifc");

  // Start by adding a wall to the file, initially leaving most attributes blank.
  IfcSchema::IfcWallStandardCase* wall = new IfcSchema::IfcWallStandardCase(
    guid(), 		// GlobalId
    0, 					// OwnerHistory
    S("Spundwand"),     // Name
    S("Automatisch erstellt durch extrusion.cpp"), 	// Description
    null, 			// ObjectType
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

  // Since the spwand_shape consists only of planar faces and straight edges it can be serialized as an
  // IfcFacetedBRep. If it would not be a polyhedron, serialise() can only be successful when linked
  // to the IFC4 model and with `advanced` set to `true` which introduces IfcAdvancedFace. It would
  // return `0` otherwise.
  IfcSchema::IfcProductDefinitionShape* object_shape = IfcGeom::serialise(spwand_shape, false);
  file.addEntity(object_shape);
  IfcSchema::IfcRepresentation* rep = *object_shape->Representations()->begin();
  rep->setContextOfItems(file.getRepresentationContext("model"));
  wall->setRepresentation(object_shape);
  // A red colour is assigned to the wall.
  file.setSurfaceColour(object_shape, 0.55 , 0.25 , 0.05);
    
  // Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy of placements for the products
  IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();

  // wall and is placed at the origin of the coordinate system.
  wall->setObjectPlacement(file.addLocalPlacement(storey_placement));

	// write the ifc File
  std::ofstream f("C1_Spundwand.ifc");
  f << file;
}


