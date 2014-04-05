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
			_viewer->addPointCloud<PointT>(cloud);
			_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
			_viewer->addCoordinateSystem(1.0);
			_viewer->initCameraParameters();
		}
	}

	void ModelViewer::draw()
	{
		init();
		_viewer->updatePointCloud<PointT>(cloud);
	}
}