#ifndef MODEL_H
#define MODEL_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <QString>
#include <QStringList>
#include <QMap>

namespace railroad {

#define PointType pcl::PointXYZ

	class Model
	{
	public:
		QStringList status() const;

		QStringList clouds() const;
		
		int cloudsIndex() const;
		void setCloudsIndex(const int & index);

	protected:
		virtual void addStatus(const QString & status) = 0;

		void resetClouds();
		virtual void addClouds(const QString & name, const QList<pcl::PointCloud<PointType>::Ptr>& clouds) = 0;

	protected:
		QStringList statusList;

		QMap<QString, QList<pcl::PointCloud<PointType>::Ptr> > cloudsMap;
		int cloudsMapIndex;

		static const QString INPUT_CLOUD;
		static const QString EUCLIDEAN_CLUSTERS;
	};
}
#endif // MODEL_H