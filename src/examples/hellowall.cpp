
#include <string>
#include <iostream>
#include <fstream>

//IFC
// #ifdef USE_IFC4
#include "../ifcparse/Ifc4.h"
// #else
// #include "../ifcparse/Ifc2x3.h"
// #endif

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
// #ifdef USE_IFC4
		, IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
// #endif
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
