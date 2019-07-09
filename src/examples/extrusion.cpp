/********************************************************************************
 *                                                                              *
 * This file is part of IfcOpenShell.                                           *
 *                                                                              *
 * IfcOpenShell is free software: you can redistribute it and/or modify         *
 * it under the terms of the Lesser GNU General Public License as published by  *
 * the Free Software Foundation, either version 3.0 of the License, or          *
 * (at your option) any later version.                                          *
 *                                                                              *
 * IfcOpenShell is distributed in the hope that it will be useful,              *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
 * Lesser GNU General Public License for more details.                          *
 *                                                                              *
 * You should have received a copy of the Lesser GNU General Public License     *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.         *
 *                                                                              *
 ********************************************************************************/

#include <string>
#include <iostream>
#include <fstream>


#ifdef USE_IFC4
#include "../ifcparse/Ifc4.h"
#else
#include "../ifcparse/Ifc2x3.h"
#endif

#include "../ifcparse/IfcBaseClass.h"
#include "../ifcparse/IfcHierarchyHelper.h"
#include "../ifcgeom/IfcGeom.h"

// Some convenience typedefs and definitions. 
typedef std::string S;
typedef IfcParse::IfcGlobalId guid;
typedef std::pair<double, double> XY;
boost::none_t const null = boost::none;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////Spundwand///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

int main() {

	// The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
	// convenience functions for working with geometry in IFC files.
	IfcHierarchyHelper file;
	file.header().file_name().name("Extrusion.ifc");
    
    //Extrusion Points
    std::vector<XY> face_points;
	face_points.push_back(XY(  0,   0));
	face_points.push_back(XY(100,   0));
	face_points.push_back(XY(200, 100));
	face_points.push_back(XY(400, 100));
	face_points.push_back(XY(500,   0));
	face_points.push_back(XY(600,   0));
	face_points.push_back(XY(600,  10));
	face_points.push_back(XY(500,  10));
	face_points.push_back(XY(400, 110));
	face_points.push_back(XY(200, 110));
	face_points.push_back(XY(100,  10));
	face_points.push_back(XY(000,  10));

  	// Start by adding a wall to the file, initially leaving most attributes blank.   
	IfcSchema::IfcWallStandardCase* extrusion = new IfcSchema::IfcWallStandardCase(
        guid(), 
        file.getSingle<IfcSchema::IfcOwnerHistory>(),
		0,                 // Name
        0,                 // Description
        0,                 // ObjectType
        0,                 // ObjectPlacement
		file.addExtrudedPolyline(face_points, 3000),  // Representation
        null               // Tag
        #ifdef USE_IFC4
        , IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
        #endif
        );
    
   	// By adding the wall to the file, a hierarchy has been automatically created that consists of the following
   	// structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
	file.addBuildingProduct(extrusion);

    //Object Placement
    IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();
	extrusion->setObjectPlacement(file.addLocalPlacement(storey_placement, 5500, 0 , 0));
    
    //Write to File
    std::ofstream f("Extrusion.ifc");
	f << file;
}









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
// 	file.addBuildingProduct(wall);
// 
// 	// By adding a wall, a hierarchy has been automatically created that consists of the following
// 	// structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
// 
// 	// Lateron changing the name of the IfcProject can be done by obtaining a reference to the 
// 	// project, which has been created automatically.
// 	file.getSingle<IfcSchema::IfcProject>()->setName("IfcOpenHouse");
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
// 		south_wall_shape, 0.75, 0.73, 0.68);
