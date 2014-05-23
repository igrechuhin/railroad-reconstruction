#include "model_viewer.h"

namespace railroad {

	ModelViewer::ModelViewer() : Model()
	{
	}

	void ModelViewer::init()
	{
		if (!_viewer)
		{
			_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
			_viewer->setBackgroundColor(0, 0, 0);
			_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
			_viewer->addCoordinateSystem(1.0);
			_viewer->initCameraParameters();
		}
	}

	void ModelViewer::draw(QList<pcl::PointCloud<PointType>::Ptr> clouds)
	{
		init();
		_viewer->removeAllPointClouds();
		int cloudID = 1;
		foreach (pcl::PointCloud<PointType>::Ptr cloud, clouds)
		{
			pcl::visualization::PointCloudColorHandlerRandom<PointType> colorHandler (cloud);
			_viewer->addPointCloud<PointType>(cloud, colorHandler, QString::number(cloudID++).toStdString());	
			//_viewer->addPointCloud<PointType>(cloud);
		}
		//_viewer->updatePointCloud<PointType>(cloud);
	}
}