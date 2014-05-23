#ifndef MODEL_VIEWER_H
#define MODEL_VIEWER_H

#include "model.h"

namespace railroad {
	class ModelViewer : virtual public Model
	{
	public:
		ModelViewer();

		void draw(QList<pcl::PointCloud<PointType>::Ptr> clouds);

	private:
		void init();

	private:
		std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
	};
}
#endif // MODEL_VIEWER_H
