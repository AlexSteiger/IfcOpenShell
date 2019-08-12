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
	stpreader.ReadFile("E13_Extrusion.stp");
	stpreader.TransferRoots();
	TopoDS_Shape extrusion_shape = stpreader.OneShape();
  std::cout << "finish reading the stp-file" << std::endl;

	std::cout << "starting reading the stl-file" << std::endl;
  StlAPI_Reader stlreader;
	QString qfilename = "V6_Verformung_cutout1.stl";
	TopoDS_Shape ausbuchtung_shape;
	stlreader.Read(ausbuchtung_shape, (Standard_CString)qfilename.toLatin1().constData());
	std::cout << "finish reading the stl-file" << std::endl;
 	
	
	std::cout << "building shell from faces... " << std::endl;
	BRep_Builder builder;
	TopoDS_Shell verformung_shell;
	builder.MakeShell(verformung_shell);
for( TopExp_Explorer ex(ausbuchtung_shape, TopAbs_FACE); ex.More(); ex.Next() )
{
	TopoDS_Face currentFace = TopoDS::Face( ex.Current() );
	BRepAdaptor_Surface brepAdaptorSurface( currentFace,Standard_True );
	builder.Add(verformung_shell, currentFace);
}

	TopoDS_Solid verformung_solid = BRepBuilderAPI_MakeSolid (verformung_shell);
	TopoDS_Shape fused = BRepAlgoAPI_Fuse(extrusion_shape,verformung_solid);
	
	// Second deformation
	std::cout << "starting reading the stl-file" << std::endl;
	qfilename = "V6_Verformung_cutout2.stl";
	TopoDS_Shape einbuchtung_shape2;
	stlreader.Read(einbuchtung_shape2, (Standard_CString)qfilename.toLatin1().constData());
	std::cout << "finish reading the stl-file" << std::endl;
	
	gp_Vec vectorToFront = gp_Vec(-0.2f,-0.1f,0);
	gp_Vec vectorToBack = gp_Vec(0.02f,0.01f,0);
	
	// Second deformation is inward, so a different algorithm is needed
	TopoDS_Shape prism  = BRepPrimAPI_MakePrism(einbuchtung_shape2,vectorToFront);
	TopoDS_Shape cuted = BRepAlgoAPI_Cut(fused,prism);
	TopoDS_Shape prism2 = BRepPrimAPI_MakePrism(einbuchtung_shape2,vectorToBack);
	TopoDS_Shape final_shape = BRepAlgoAPI_Fuse(cuted,prism2);
	
// 	//This tool tries to unify faces and edges of the shape which lie on the same geometry
// 	ShapeUpgrade_UnifySameDomain unif(fused2, true, true, false);
// 	unif.Build();
// 	TopoDS_Shape fixed = unif.Shape();
	
	//write Files
	STEPControl_Writer OCC_writer;
	OCC_writer.Transfer(final_shape,STEPControl_AsIs);
 	OCC_writer.Write("F1_Spundwand.stp");

}


