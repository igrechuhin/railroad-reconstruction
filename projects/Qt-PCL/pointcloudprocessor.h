#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <QObject>
#include <QUrl>

namespace railroad {

#define PointT pcl::PointXYZ

class PointCloudProcessor : public QObject
{
    Q_OBJECT
	
	Q_PROPERTY(bool valid READ valid WRITE setValid NOTIFY validChanged)
	Q_PROPERTY(QString status READ status WRITE setStatus NOTIFY statusChanged)

public:
    explicit PointCloudProcessor(QObject *parent = 0);

    Q_INVOKABLE void readModel(const QUrl & url);

	void setValid(const bool & valid);
	bool valid() const;

	void setStatus(const QString & status);
	QString status() const;

private:
	bool readPCD(QString & fileURL);
	bool readCSV(QString & fileURL);

	void resetCloud();

signals:
	void validChanged();
	void statusChanged();

public slots:

private:
	pcl::PointCloud<PointT>::Ptr _cloud;
	bool _cloudValid;

	QString _status;
};

}

#endif // POINTCLOUDPROCESSOR_H
