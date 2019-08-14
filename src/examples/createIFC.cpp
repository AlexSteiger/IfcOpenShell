#include <iostream>
//OCC:
#include "STEPControl_Reader.hxx"
#include <TopoDS_Shape.hxx>
//IFC:
#include "../ifcparse/Ifc4.h"
#include "../ifcparse/IfcBaseClass.h"
#include "../ifcparse/IfcHierarchyHelper.h"
#include "../ifcgeom/IfcGeom.h"

#if USE_VLD
#include <vld.h>
#endif

// Some convenience typedefs and definitions. 
//typedef std::string S;
//typedef IfcParse::IfcGlobalId guid;
//boost::none_t const null = boost::none;

int main(int argc, char *argv[]) {

	std::string filename;
	if (!argv[1])
	{
		filename = "F1_Spundwand.stp";
		std::cout << "kein Inputdatei angegeben. " << filename << " wird aufgerufen.\n";
	}
	if (argv[1])
	{
		std:: cout << "Inputdatei angegeben" << std::endl;
		filename = argv[1];
	}
	
	std::cout << "starting reading the stp-file" << std::endl;
	STEPControl_Reader stpreader;
	stpreader.ReadFile(filename.c_str());
	stpreader.TransferRoots();
	TopoDS_Shape spwand_shapeOCC = stpreader.OneShape();
  std::cout << "finish reading the stp-file" << std::endl;

  //////////////////////////////////
  //////////IFC STARTS HERE/////////
  //////////////////////////////////
    
  // The IfcHierarchyHelper is a subclass of the regular IfcFile that provides several
  // convenience functions for working with geometry in IFC files.
  IfcHierarchyHelper file;
	
	// Writes the filename in the Header of the IFC file.
  file.header().file_name().name("Spundwand.ifc");

  // Define a IfcOwnerHistory (optional) by adding a IfcPersonAndOrganization & IfcApplication
  IfcSchema::IfcPersonAndOrganization* pando = new IfcSchema::IfcPersonAndOrganization
  (
    new IfcSchema::IfcPerson
    (
      boost::none,              // Identification
      std::string("Steiger"),   // FamilyName
      std::string("Alexander"), // GivenName
      boost::none,              // MiddleNames
      boost::none,              // PrefixTitles
      boost::none,              // SuffixTitles
      boost::none,              // Roles
      boost::none               // Addresses
    ),
    new IfcSchema::IfcOrganization
    (
      boost::none,                                  // Identification
      std::string("Uni Rostock | Fraunhofer IGD"),  // Name
      boost::none,                                  // Description
      boost::none,                                  // Roles
      boost::none                                   // Addresses
    ),
    boost::none	// Roles
  );
  IfcSchema::IfcApplication* appl = new IfcSchema::IfcApplication
  (
    pando->TheOrganization(),   // ApplicationDeveloper
    "",                         // Version
    "",                         // ApplicationFullName
    ""                          // ApplicationIdentifier
  );
  file.addEntity(pando);
  file.addEntity(appl);
  IfcSchema::IfcOwnerHistory* history = new IfcSchema::IfcOwnerHistory
  (
    pando,                                            // OwningUser
    appl,                                             // OwningApplication
    boost::none,                                      // State
    Ifc4::IfcChangeActionEnum::IfcChangeAction_ADDED, // ChangeAction
    boost::none,                                      // LastModifiedDate
    pando,                                            // LastModifyingUser
    appl,                                             // LastModifyingApplication
    std::time(0)                                      // CreationDate
  );
  file.addEntity(history);

  // Start by adding a wall to the file, initially leaving most attributes blank.
  IfcSchema::IfcWall* spwand = new IfcSchema::IfcWall
  (
    IfcParse::IfcGlobalId(),                      // GlobalId
    file.getSingle<IfcSchema::IfcOwnerHistory>(), // OwnerHistory
    std::string("Spundwand"),                     // Name
    std::string("Mönckebergkai (Ausschnitt)"),    // Description
    boost::none,                                  // ObjectType
    0,                                            // ObjectPlacement
    0,                                            // Representation
    boost::none,                                  // Tag
		IfcSchema::IfcWallTypeEnum::IfcWallType_STANDARD
  );
  // By adding a wall, a hierarchy has been automatically created that consists of the following
  // structure: IfcProject > IfcSite > IfcBuilding > IfcBuildingStorey > IfcWall
  file.addBuildingProduct(spwand);

  // Since the spwand_shapeOCC consists only of planar faces and straight edges it can be serialized as an
  // IfcFacetedBRep. If it would not be a polyhedron, serialise() can only be successful when linked
  // to the IFC4 model and with `advanced` set to `true` which introduces IfcAdvancedFace. It would
  // return `0` otherwise.
  IfcSchema::IfcProductDefinitionShape* product_shape = IfcGeom::serialise(spwand_shapeOCC, false);
  file.addEntity(product_shape);
  IfcSchema::IfcRepresentation* rep = *product_shape->Representations()->begin();
  rep->setContextOfItems(file.getRepresentationContext("model"));
  spwand->setRepresentation(product_shape);
  // A red colour is assigned to the wall.
  file.setSurfaceColour(product_shape, 0.57 , 0.24 , 0.15);
    
  // Obtain a reference to the placement of the IfcBuildingStorey in order to create a hierarchy of placements for the products
  IfcSchema::IfcObjectPlacement* storey_placement = file.getSingle<IfcSchema::IfcBuildingStorey>()->ObjectPlacement();

  // wall is placed at the origin of the coordinate system.
  spwand->setObjectPlacement(file.addLocalPlacement(storey_placement));

	// write the ifc File
  std::ofstream f("Spundwand.ifc");
  f << file;
	
	

}


