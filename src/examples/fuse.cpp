#include <string>
#include <iostream>
#include <fstream>

#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS_Shape.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

int main() 
{
  // Read in the STP-File (Base Model)
	STEPControl_Reader stpreader;
	stpreader.ReadFile("B13_Extrusion.stp");
	stpreader.TransferRoots();
	TopoDS_Shape baseModel = stpreader.OneShape();

  // Read in the STL-File (Deformation)
  StlAPI_Reader stlreader;
 	Standard_CString filename;
	filename = "D6_Verformung_cutout.stl";
	TopoDS_Shape deformation;
	stlreader.Read(deformation, filename);
	
  // Algorithm to fuse the deformation with the base model
	gp_Vec vectorToFront = gp_Vec(-200.0f,-100.0f,0);
	gp_Vec vectorToBack = gp_Vec(20.0f,10.0f,0);
	TopoDS_Shape prism  = BRepPrimAPI_MakePrism(deformation,vectorToFront);
	TopoDS_Shape cuted = BRepAlgoAPI_Cut(baseModel,prism);
	TopoDS_Shape prism2 = BRepPrimAPI_MakePrism(deformation,vectorToBack);
	TopoDS_Shape fusedShape = BRepAlgoAPI_Fuse(cuted,prism2);
	
	//This tool tries to unify faces and edges of the shape which lie on the same geometry
	ShapeUpgrade_UnifySameDomain unif(fusedShape, true, true, false);
	unif.Build();
	TopoDS_Shape fixedShape = unif.Shape();
	
	//write Files
	STEPControl_Writer OCC_writer;
	OCC_writer.Transfer(fixedShape,STEPControl_AsIs);
 	OCC_writer.Write("F1_Spundwand.stp");
}
