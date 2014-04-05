#ifndef MODEL_H
#define MODEL_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <QString>
#include <QStringList>

namespace railroad {

#define PointT pcl::PointXYZ

	class Model
	{
	public:
		Model();

		bool valid() const;
		QStringList status() const;

	protected:
		void resetCloud();

		virtual void setValid(const bool & valid) = 0;
		virtual void addStatus(const QString & status) = 0;

	protected:
		pcl::PointCloud<PointT>::Ptr cloud;
		bool cloudValid;

		QStringList statusList;
	};

}
#endif // MODEL_H