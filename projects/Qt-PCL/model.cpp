#include "Model.h"

namespace railroad {

	Model::Model()
		: cloud(new pcl::PointCloud<PointType>)
		, cloudValid(false)
		, cloudFiltered(new pcl::PointCloud<PointType>)
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
