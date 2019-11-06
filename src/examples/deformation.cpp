#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <pcl/common/common.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char *argv[])
{
  // Read in the PCD-File
	std::string filename;
	if (!argv[1])
	{
		filename = "Spundwand.pcd";
		std::cout << "Keine Inputdatei als Argument angegeben.";
		std::cout << filename << " wird aufgerufen.\n";
	}
	if (argv[1])
	{
		std:: cout << "Inputdatei angegeben\n";
		filename = argv[1];
	}
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (filename, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud_input);
  std::cout << "Input has: " << cloud_input->points.size () << " data points.\n";
  
  // Get the correct scale of the cloud 
  pcl::PointCloud<pcl::PointXYZ>::PointType cloud_pkt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t j = 0; j < cloud_input->points.size() ; ++j)
    {
    cloud_pkt.x = cloud_input->points[j].x*1000; 
    cloud_pkt.y = cloud_input->points[j].y*1000; 
    cloud_pkt.z = cloud_input->points[j].z*1000;
    cloud->push_back(cloud_pkt);
    }
  pcl::io::savePCDFile ("D0_cloud_input.pcd", *cloud);
  
  // Some informations about the cloud
	pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt); 
  std::cout << "Dimensions of Input (";
  std::cout << cloud->points.size () << " data points; Unit: [mm]):\n";
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
	
  // Statistical outlier removal
  int nrpoints = cloud->points.size();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (60);
  sor.setStddevMulThresh (4);
  sor.setNegative (false);
  sor.filter (*cloud);
  std::cout << nrpoints - cloud->points.size () << " outliers  removed. ";
  std::cout << cloud->points.size() << " points left\n"; 
  pcl::io::savePCDFile ("D1_cloud_filtered.pcd", *cloud);

  // MovingLeastSquares
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setInputCloud (cloud);
  mls.setComputeNormals (false);
  mls.setSearchRadius (100);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder (3);
  mls.process (*cloud_smoothed);
  std::cout << "cloud_smoothed has: " << cloud_smoothed->points.size() << " points.\n";
  pcl::io::savePCDFile ("D2_cloud_smoothed.pcd", *cloud_smoothed);
	
	// Downsample the Cloud with a Voxelgrid filter
  nrpoints = cloud_smoothed->points.size();
	pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud_smoothed);
  vox.setLeafSize (30.0f, 30.0f, 30.0f);
  vox.filter (*cloud);
	pcl::io::savePCDFile ("D3_cloud_downsampled.pcd", *cloud);
	std::cout << "Downsampling:" <<  nrpoints - cloud->points.size();
	std::cout << " points where removed.\n";
	
  // This part here because otherwise an Error occurs for the following normal estimation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob2;
  pcl::io::loadPCDFile ("D3_cloud_downsampled.pcd", cloud_blob2);
  pcl::fromPCLPointCloud2 (cloud_blob2, *cloud_temp);
	
  // Normal Estimation
  std::cout << "starting normal estimation\n";
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
  ne.setNumberOfThreads (8);
  ne.setInputCloud (cloud_temp);
  ne.setRadiusSearch (300);
  ne.setViewPoint(10000,-375000,-5000);
  ne.compute (*normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloud_temp, *normals, *cloud_normals);
  pcl::io::savePCDFile ("D4_cloud_normals.pcd", *cloud_normals);
  
  //Poisson
	auto start = std::chrono::high_resolution_clock::now();
  pcl::Poisson<pcl::PointNormal> poisson;  
  // Set the maximum depth of the tree that will be used for surface reconstruction.
  poisson.setDepth (7);
	std::cout << "starting Poisson with tree depth " << poisson.getDepth();
	std::cout << " and " << cloud_normals->points.size() << " points.\n";
  // Set the minimum number of sample points that should fall within an octree node
  poisson.setSamplesPerNode(6);
  poisson.setScale(1);
  poisson.setInputCloud (cloud_normals);  
  pcl::PolygonMesh mesh;  
  poisson.reconstruct (mesh);
  pcl::io::savePolygonFileSTL("D5_Poisson.stl", mesh); 
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Poisson took: " << elapsed.count() << " s\n";
  return 0;
}



//#include <pcl/surface/marching_cubes_hoppe.h>
//   // MarchingCubesHoppe: Doesn't work so well
//   std::cout << "begin marching cubes Hoppe reconstruction" << std::endl;
//   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
//   pcl::MarchingCubesHoppe<pcl::PointNormal> hoppe;
//   hoppe.setIsoLevel (0); //between 0 and 1
//   hoppe.setGridResolution (100, 100, 100);  //je höher, desto mehr Polygone
//   hoppe.setPercentageExtendGrid (0.01f);
//   hoppe.setInputCloud (cloud_normals);
//   hoppe.reconstruct (*triangles);
//   std::cout << triangles->polygons.size() << " triangles created" << std::endl;
//   pcl::io::savePolygonFileSTL("D5_MCH_Iso0_Grid100_PEG001.stl", *triangles);
//   

//   // Viewer
// #include <pcl/visualization/pcl_visualizer.h>
// 	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (255,255,255);
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_normals, 30, 30, 30);
// 	viewer->addPointCloud<pcl::PointNormal> (cloud_normals, single_color, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
// 	//addPointCloudNormals(cloud, level, scale, id)
//   viewer->addPointCloudNormals<pcl::PointNormal> (cloud_normals, 1, 0.1,  "normals");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 90,0,0, "normals");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//     while (!viewer->wasStopped ()) //Calls the interactor and updates the screen once. 
//   {
//     viewer->spinOnce (100);
//     //std::this_thread::sleep_for(100ms);
//   }

//#include <pcl/surface/gp3.h>
//     // Greedy Projection: Doesn't work so well
//   std::cout << "starting greedy projection for greedy projection" << std::endl;
//   // Create search tree*
//   pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
//   tree3->setInputCloud (cloud_normals); 
//   // Initialize objects
//   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//   pcl::PolygonMesh triangles;
//   // Set the maximum distance between connected points (maximum edge length)
//   gp3.setSearchRadius (1000);
//   // Set typical values for the parameters
//   gp3.setMu (2);  //Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point (this will make the algorithm adapt to different point densities in the cloud).
//   gp3.setMaximumNearestNeighbors (100); //nnn
//   gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
//   gp3.setMinimumAngle(M_PI/18); // 10 degrees
//   gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//   gp3.setNormalConsistency(false);
//   // Get result
//   gp3.setInputCloud (cloud_normals);  
//   gp3.setSearchMethod (tree3);
//   mls.setSearchRadius (100);
//   gp3.reconstruct (triangles);
//   // Additional vertex information
//   std::vector<int> parts = gp3.getPartIDs();
//   std::vector<int> states = gp3.getPointStates();
//   pcl::io::savePolygonFileSTL("D5_GreedyProjection.stl",triangles);
  
//   // MarchingCubesHoppe: Doesn't work so well
//   std::cout << "begin marching cubes Hoppe reconstruction" << std::endl;
//   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
//   pcl::MarchingCubesHoppe<pcl::PointNormal> hoppe;
//   hoppe.setIsoLevel (0); //between 0 and 1
//   hoppe.setGridResolution (100, 100, 100);  //je höher, desto mehr Polygone
//   hoppe.setPercentageExtendGrid (0.01f);
//   hoppe.setInputCloud (cloud_normals);
//   hoppe.reconstruct (*triangles);
//   std::cout << triangles->polygons.size() << " triangles created" << std::endl;
//   pcl::io::savePolygonFileSTL("MCH_Iso0_Grid100_PEG001.stl", *triangles);
