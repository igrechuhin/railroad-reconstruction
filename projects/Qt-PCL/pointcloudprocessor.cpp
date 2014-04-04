#include "pointcloudprocessor.h"

#include <QFileInfo>
#include <QTextStream>
#include <vector>

namespace {
	bool parseToFloat(const QString & str, float & val)
	{
		bool ok;
		val = str.toFloat(&ok);
		Q_ASSERT(ok);
		return ok;
	}
}

namespace railroad {

PointCloudProcessor::PointCloudProcessor(QObject *parent) :
    QObject(parent),
	_cloudValid(false)
{
}

void PointCloudProcessor::resetCloud()
{
	_cloud.reset(new pcl::PointCloud<PointT>);
	setValid(false);
}

void PointCloudProcessor::readModel(const QUrl & url)
{
	QString fileURL(url.toLocalFile());
    if (url.isValid())
	{
		resetCloud();
		setStatus(tr("Reading file: ") + fileURL);
		QString fileType = QFileInfo(fileURL).suffix();
		if (fileType == QStringLiteral("pcd"))
		{
			setValid(readPCD(fileURL));
		}
		else if (fileType == QStringLiteral("csv"))
		{
			setValid(readCSV(fileURL));
		}

		setStatus((valid() ? tr("File read: ") : tr("Couldn't read file: ")) + fileURL);
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud (_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
	else
	{
		setStatus(tr("Invalid file url: ")+fileURL);
	}	
}

void PointCloudProcessor::setValid(const bool & valid)
{
	if (valid != _cloudValid) {
		_cloudValid = valid;
		emit validChanged();
	}
}

bool PointCloudProcessor::valid() const
{
	return _cloudValid;
}

void PointCloudProcessor::setStatus(const QString & status)
{
	if (status != _status) {
		_status = status;
		emit statusChanged();
	}
}

QString PointCloudProcessor::status() const
{
	return _status;
}

bool PointCloudProcessor::readPCD(QString & fileURL)
{
	return pcl::io::loadPCDFile<PointT> (fileURL.toStdString(), *_cloud) != -1;
}

bool PointCloudProcessor::readCSV(QString & fileURL)
{
	QFile file(fileURL);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		std::vector<PointT, Eigen::aligned_allocator<PointT> > points;
		QTextStream in(&file);
		QRegExp separator(QStringLiteral(","));
		float x,y,z;

		while (!in.atEnd()) {
			QString line = in.readLine();
			QStringList list = line.split(separator);
			if (!(parseToFloat(list[0], x) && parseToFloat(list[1], y) && parseToFloat(list[2], z)))
			{
				setStatus(tr("Invalid number in file: ")+line);
				return false;
			}
			points.push_back(PointT(x, y, z));
		}

		_cloud->width = points.size();
		_cloud->height = 1;
		_cloud->points = points;
		_cloud->is_dense = true;

		return true;
	}
	else
	{
		setStatus(tr("Can not open file."));
		return false;
	}
}

}
