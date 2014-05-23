#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H

#include "model_io.h"
#include "model_viewer.h"

#include <QObject>

namespace railroad {

	class PointCloudProcessor : public QObject, public ModelIO, public ModelViewer
	{
		Q_OBJECT

		//Q_PROPERTY(bool valid READ valid WRITE setValid NOTIFY validChanged)
		Q_PROPERTY(QStringList status READ status NOTIFY statusChanged)

		Q_PROPERTY(QStringList clouds READ clouds NOTIFY cloudsChanged)
		Q_PROPERTY(int cloudsIndex READ cloudsIndex WRITE setCloudsIndex NOTIFY cloudsIndexChanged)
	public:
		explicit PointCloudProcessor(QObject * parent = 0);

		Q_INVOKABLE void readModel(const QUrl & url);
		Q_INVOKABLE void writeModel(const QUrl & url, const QString & cloudName);

		Q_INVOKABLE void euclideanClusterExtraction(const QString & cloudName);

		Q_INVOKABLE void draw(const QString & cloudName);

		virtual void addStatus(const QString & status);

		virtual void addClouds(const QString & name, const QList<pcl::PointCloud<PointType>::Ptr>& clouds);

	private:
		void _doEuclideanClusterExtraction(const QList<pcl::PointCloud<PointType>::Ptr> & clouds);

	signals:
		void statusChanged();
		void cloudsChanged();
		void cloudsIndexChanged();
	};
}

#endif // POINTCLOUDPROCESSOR_H
