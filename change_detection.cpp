#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/visualization/cloud_viewer.h>

float voxel_size = 0.1;
int count = 0;
int counttarget = 0;

void XOR(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//Empty Box
	pcl::VoxelGridCovariance<pcl::PointXYZ> cell_model;
	cell_model.setLeafSize(voxel_size,voxel_size,voxel_size);
	cell_model.setInputCloud(cloud_model);
	cell_model.filter(true);
	const std::map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf> &leaves_model = cell_model.getLeaves();
	//Target Box
	pcl::VoxelGridCovariance<pcl::PointXYZ> cell_target;
	cell_target.setLeafSize(voxel_size,voxel_size,voxel_size);
	cell_target.setInputCloud(cloud_target);
	cell_target.filter(true);
	const std::map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf> &leaves_target = cell_target.getLeaves();
	//XOR
	for(std::map<size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf>::const_iterator it = leaves_target.begin(); it != leaves_target.end(); ++it)
	{
		const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf &leaf_target = it->second;
		Eigen::Vector3f p_target = leaf_target.getMean().cast<float>();;
		const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *leaf_model = cell_model.getLeaf(p_target);
		if(leaf_model == NULL){
			std::cout << "in point" << std::endl;
            count++;
			//Eigen::Vector3f p_model = leaf_model->getMean().cast<float>();;
			std::cout << leaf_target.getPointCount() << std::endl;
			//std::cout << leaf_model->getPointCount() << std::endl;
			std::cout << p_target << std::endl;
			//std::cout << p_model << std::endl;
		}
        counttarget++;
	}
    std::cout << count << std::endl;
    std::cout << counttarget << std::endl;
}

pcl::PointCloud<pcl::PointXYZ> difference_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test)
{
    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

    double resolution = 0.00001;//Octreeの解像度を指定
   
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);//Octreeを作成
   
    octree.setInputCloud (cloud_base);//元となる点群を入力
    octree.addPointsFromInputCloud ();

    octree.switchBuffers ();//バッファの切り替え

    octree.setInputCloud (cloud_test);//比較対象の点群を入力
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;//

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ> );//出力先


    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);   

    int n = 0;//差分点群の数を保存する
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        n++;
    }
    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    return *cloud_diff;
}

int main(int argc, char **argv)
{
    //srand ((unsigned int) time (NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ> );

    pcl::io::loadPCDFile("bunny.pcd", *cloud_model);
	pcl::io::loadPCDFile("test_pcd.pcd", *cloud_target);
	XOR(cloud_target,cloud_model);

    //pcl::io::savePCDFileBinary("difference.pcd", difference_extraction(cloud_base, cloud_test));

    // pcl::PointCloud<pcl::PointXYZ> cloud;

    // // Fill in the cloud data
    // cloud.width = 100;
    // cloud.height = 100;
    // cloud.is_dense = false;
    // cloud.points.resize(cloud.width * cloud.height);

    // for (size_t i = 0; i < cloud.points.size(); ++i)
    // {
    //     cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }

    // pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    // std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    // for (size_t i = 0; i < cloud.points.size(); ++i)
    //     std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud_target);
	while (!viewer.wasStopped ())
	{
	}
    return (0);
}
