#include <pcl/common/common.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
//#include <pcl/surface/marching_cubes.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h> // for savePolygonFileSTL
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

//IFC
#include "../ifcparse/IfcFile.h"

#if USE_VLD
#include <vld.h>
#endif

int
main (int argc, char** argv)
{
  
  // Input Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
  std::cout << "Input has: " << cloud_input->points.size () << " data points." << std::endl;
  pcl::io::savePCDFile ("V0_cloud_input.pcd", *cloud_input);
	pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_input, minPt, maxPt); 
  std::cout << "Dimensions of Input (";
  std::cout << cloud_input->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
	
  // MovingLeastSquares 
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //(<Input_Cloud, Output_Cloud>)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
  mls.setInputCloud (cloud_input);
  mls.setComputeNormals (false);
  mls.setSearchRadius (0.1);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder (3);
  mls.process (*cloud_smoothed);
  std::cout << "cloud_smoothed has: " << cloud_smoothed->points.size () << " points." << std::endl;
  pcl::io::savePCDFile ("V1_cloud_smoothed.pcd", *cloud_smoothed);
	
  // Statistical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_smoothed);
  sor2.setMeanK (60);
  sor2.setStddevMulThresh (3);
  sor2.setNegative (false);
  sor2.filter (*cloud_filtered);
  std::cout << cloud_smoothed->points.size() - cloud_filtered->points.size () << " outliers where removed. ";
  std::cout << cloud_filtered->points.size() << " points left" << std::endl; 
  pcl::io::savePCDFile ("V2_cloud_filtered.pcd", *cloud_filtered);
	
  // This part is only here because otherwise this Error occurs for the following normal estimation:
  // surface_reconstruction: /build/pcl-OilVEB/pcl-1.8.1+dfsg1/kdtree/include/pcl/kdtree/impl/kdtree_flann.hpp:172: int pcl::KdTreeFLANN<PointT, Dist>::radiusSearch(const PointT&, double, std::vector<int>&, std::vector<float>&, unsigned int) const [with PointT = pcl::PointXYZ; Dist = flann::L2_Simple<float>]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"' failed.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob2;
  pcl::io::loadPCDFile ("V2_cloud_filtered.pcd", cloud_blob2);
  pcl::fromPCLPointCloud2 (cloud_blob2, *cloud_temp);
  std::cout << "Temp has: " << cloud_temp->points.size () << " data points." << std::endl;

  // Normal Estimation
  // Ziel: dass die Normalen in eine Richtung schauen(?)
  std::cout << "starting normal estimation" << std::endl;
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  ne.setNumberOfThreads (8);
  ne.setInputCloud (cloud_temp);
  ne.setRadiusSearch (0.1);
  ne.setViewPoint(20,-150,-5);
  ne.compute (*cloud_normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloud_temp, *cloud_normals, *cloud_smoothed_normals);
  pcl::io::savePCDFile ("V3_cloud_smoothed_normals.pcd", *cloud_smoothed_normals);
	
//   // Viewer
//   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPointCloud<pcl::PointNormal> (cloud_smoothed_normals, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addPointCloudNormals<pcl::PointNormal> (cloud_smoothed_normals, 10, 0.5,"normals");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//     while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce (100);
//     //std::this_thread::sleep_for(100ms);
//   }
  
  //Poisson - Sensitive to noise, but still best outcome
  std::cout << "starting Poisson" << std::endl;
  pcl::Poisson<pcl::PointNormal> poisson;  
  // Set the maximum depth of the tree that will be used for surface reconstruction. Higher -> more Details
  poisson.setDepth (7);  // 7 is ok
  // Set the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density.  For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction. 
  poisson.setSamplesPerNode(5);
  poisson.setScale(1);
  poisson.setInputCloud (cloud_smoothed_normals);  
  pcl::PolygonMesh mesh;  
  poisson.reconstruct (mesh);
  //std::cout << poisson.getDegree() << std::endl; //what is Degree for?
  pcl::io::savePolygonFileSTL("V4_Poisson.stl", mesh); 

	
  
  
  
   
   
  
//   IfcParse::IfcFile file;
//   file.Init("Mauer.ifc");
//   
//   IfcSchema::IfcBuildingElement::list::ptr elements = file.entitiesByType<IfcSchema::IfcBuildingElement>();
//   std::cout << "Found " << elements->size() << " elements in " << "Mauer.ifc" << ":" << std::endl;
//   
//   
//   for ( IfcSchema::IfcBuildingElement::list::it it = elements->begin(); it != elements->end(); ++ it ) {
// 		
// 		const IfcSchema::IfcBuildingElement* element = *it;
// 		std::cout << element->entity->toString() << std::endl;
// 		
// 		if ( element->is(IfcSchema::IfcWindow::Class()) ) {
// 			const IfcSchema::IfcWindow* window = (IfcSchema::IfcWindow*)element;
// 			
// 			if ( window->hasOverallWidth() && window->hasOverallHeight() ) {
// 				const double area = window->OverallWidth()*window->OverallHeight();
// 				std::cout << "The area of this window is " << area << std::endl;
// 			}
// 		}
// 
// 	}
  
  
  
  
  
  
  return 0;
}























//   // Greedy Projection: Doesn't work so well
//   std::cout << "starting greedy projection for greedy projection" << std::endl;
//   // Create search tree*
//   pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
//   tree3->setInputCloud (cloud_smoothed_normals); 
//   // Initialize objects
//   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//   pcl::PolygonMesh triangles;
//   // Set the maximum distance between connected points (maximum edge length)
//   gp3.setSearchRadius (10);
//   // Set typical values for the parameters
//   gp3.setMu (2);  //Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point (this will make the algorithm adapt to different point densities in the cloud).
//   gp3.setMaximumNearestNeighbors (100); //nnn
//   gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//   gp3.setMinimumAngle(M_PI/18); // 10 degrees
//   gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//   gp3.setNormalConsistency(false);
//   // Get result
//   gp3.setInputCloud (cloud_smoothed_normals);  
//   gp3.setSearchMethod (tree3);
//   mls.setSearchRadius (0.1);
//   gp3.reconstruct (triangles);
//   // Additional vertex information
//   std::vector<int> parts = gp3.getPartIDs();
//   std::vector<int> states = gp3.getPointStates();
//   pcl::io::saveVTKFile ("SR4_triangles.vtk",triangles);
  
//   // MarchingCubesHoppe: Doesn't work so well
//   std::cout << "begin marching cubes Hoppe reconstruction" << std::endl;
//   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
//   pcl::MarchingCubesHoppe<pcl::PointNormal> hoppe;
//   hoppe.setIsoLevel (0); //between 0 and 1
//   hoppe.setGridResolution (100, 100, 100);  //je höher, desto mehr Polygone
//   hoppe.setPercentageExtendGrid (0.01f);
//   hoppe.setInputCloud (cloud_smoothed_normals);
//   hoppe.reconstruct (*triangles);
//   std::cout << triangles->polygons.size() << " triangles created" << std::endl;
//   pcl::io::savePolygonFileSTL("MCH_Iso0_Grid100_PEG001.stl", *triangles);
