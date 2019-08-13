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

#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

#include <Standard_Version.hxx>

#include "../ifcparse/Ifc4.h"

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

int x = 0;
int j = 0;
// The creation of Nurbs-surface for the IfcSite mesh, to be implemented lateron
void createGroundShape(TopoDS_Shape& shape);

int main() {

	// The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
	// convenience functions for working with geometry in IFC files.
	IfcHierarchyHelper file;
	file.header().file_name().name("IfcOpenHouse.ifc");
    
    for (int i = 0; i < 20; ++i) {
    x = x + 1;
        j = 0;
        for (int k = 0; k < 10; ++k) {
        j = j + 1;
        // Start by adding a wall to the file, initially leaving most attributes blank.
        IfcSchema::IfcWallStandardCase* Mauer = new IfcSchema::IfcWallStandardCase(
            guid(), 			// GlobalId
            0, 					// OwnerHistory
            0,               	// Name
            S("Das ist eine Wand"), 				// Description
            null, 				// ObjectType
            0, 					// ObjectPlacement
            0, 					// Representation
            null				// Tag
            ,IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
        );
        file.addBuildingProduct(Mauer);

        // By adding a wall, a hierarchy has been automatically created that consists of the following
        // structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall

        // Lateron changing the name of the IfcProject can be done by obtaining a reference to the 
        // project, which has been created automatically.
        file.getSingle<IfcSchema::IfcProject>()->setName("IfcOpenHouse");

        // An IfcOwnerHistory has been initialized as well, which should be assigned to the wall.
        Mauer->setOwnerHistory(file.getSingle<IfcSchema::IfcOwnerHistory>());

        // The wall will be shaped as a box, with the dimensions specified in millimeters. The resulting
        // product definition will consist of both a body representation as well as an axis representation
        // that runs over the centerline of the box in the X-axis.
        IfcSchema::IfcProductDefinitionShape* wall_shape = file.addAxisBox(18000, 2000, 9000);

        // Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy
        // of placements for the products
        IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();

        // The shape has to be assigned to the representation of the wall and is placed at the origin
        // of the coordinate system.
        Mauer->setRepresentation(wall_shape);
        Mauer->setObjectPlacement(file.addLocalPlacement(storey_placement, 19500 * j, 0, 10500 * x));

        // A pale white colour is assigned to the wall.
        IfcSchema::IfcPresentationStyleAssignment* wall_colour = file.setSurfaceColour(
            wall_shape, 0.75, 0.73, 0.68);
        
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////
	// According to the Ifc2x3 schema an IfcWallStandardCase needs to have an IfcMaterialLayerSet
	// assigned. Note that this material definition is independent of the surface styles we have 
	// been assigning to the walls already. The surface styles determine the colour in the 
	// '3D viewport' of most applications.
	// Some BIM authoring applications, such as Autodesk Revit, ignore the geometrical representation
	// by and large and construct native walls using the layer thickness and reference line offset 
	// provided here.
#ifdef USE_IFC4
	IfcSchema::IfcMaterial* material = new IfcSchema::IfcMaterial("Brick", null, null);
#else
	IfcSchema::IfcMaterial* material = new IfcSchema::IfcMaterial("Brick");
#endif
	IfcSchema::IfcMaterialLayer* layer = new IfcSchema::IfcMaterialLayer(
		material, 
		360, 
		null
#ifdef USE_IFC4
		, null
		, null 
		, null
		, null
#endif
	);
	IfcSchema::IfcMaterialLayer::list::ptr layers (new IfcTemplatedEntityList<IfcSchema::IfcMaterialLayer>());
	layers->push(layer);
	IfcSchema::IfcMaterialLayerSet* layer_set = new IfcSchema::IfcMaterialLayerSet(
		layers, 
		S("Wall")
#ifdef USE_IFC4
		, null
#endif
	);
	IfcSchema::IfcMaterialLayerSetUsage* layer_usage = new IfcSchema::IfcMaterialLayerSetUsage(
		layer_set,
		IfcSchema::IfcLayerSetDirectionEnum::IfcLayerSetDirection_AXIS2,
		IfcSchema::IfcDirectionSenseEnum::IfcDirectionSense_POSITIVE,
		-180
#ifdef USE_IFC4
		, null
#endif
	);

	IfcSchema::IfcRelAssociatesMaterial* associates_material = new IfcSchema::IfcRelAssociatesMaterial(
		guid(),
		file.getSingle<IfcSchema::IfcOwnerHistory>(), 
		null, 
		null,
#ifdef USE_IFC4
		file.entitiesByType<IfcSchema::IfcWallStandardCase>()->generalize(),
#else
		file.entitiesByType<IfcSchema::IfcWallStandardCase>()->as<IfcSchema::IfcRoot>(),
#endif
		layer_usage);

	file.addEntity(material);
	file.addEntity(layer);
	file.addEntity(layer_set);	
	file.addEntity(layer_usage);
    file.addEntity(associates_material);
    
    std::ofstream f("HelloWalls.ifc");
	f << file;

}
