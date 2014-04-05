#include "Model.h"

namespace railroad {

	Model::Model()
		: cloud(new pcl::PointCloud<PointT>),
		  cloudValid(false)
	{
	}

	void Model::resetCloud()
	{
		cloud->clear();
		setValid(false);
	}

	bool Model::valid() const
	{
		return cloudValid;
	}

	QStringList Model::status() const
	{
		return statusList;
	}
}
