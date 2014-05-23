#include "model_io.h"

#include <QFileInfo>
#include <QTextStream>
#include <QString>

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

	ModelIO::ModelIO() : Model()
	{
	}

	bool ModelIO::readModel(const QUrl & url)
	{
		QString fileURL(url.toLocalFile());
		addStatus(QObject::tr("Reading file: ") + fileURL);
		if (url.isValid())
		{
			resetClouds();
			pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

			QString fileType = QFileInfo(fileURL).suffix();
			if (fileType == QStringLiteral("pcd"))
			{
				readPCD(fileURL, cloud);
			}
			else if (fileType == QStringLiteral("csv"))
			{
				readCSV(fileURL, cloud);
			}

			if (cloud->empty())
			{
				addStatus(QObject::tr("Couldn't read file: ") + fileURL);
			}
			else
			{
				QList<pcl::PointCloud<PointType>::Ptr> cloudsList;
				cloudsList.append(cloud);
				addClouds(INPUT_CLOUD, cloudsList);
				addStatus(QObject::tr("File read: ") + fileURL);
				return true;
			}
		}
		else
		{
			addStatus(QObject::tr("Invalid file url: ") + fileURL);
		}
		return false;
	}

	void ModelIO::writeModel(const QUrl & url, const QString & cloudName)
	{
		QString fileURL(url.toLocalFile());
		addStatus(QObject::tr("Writing file: ") + fileURL);
		if (url.isValid())
		{
			auto clouds = cloudsMap[cloudName];
			writePCD(fileURL, clouds);
		}
	}

	bool ModelIO::readPCD(QString & fileURL, pcl::PointCloud<PointType>::Ptr cloud)
	{
		return pcl::io::loadPCDFile<PointType>(fileURL.toStdString(), *cloud) != -1;
	}

	bool ModelIO::readCSV(QString & fileURL, pcl::PointCloud<PointType>::Ptr cloud)
	{
		QFile file(fileURL);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream in(&file);
			QRegExp separator(QStringLiteral(","));
			float x,y,z;

			while (!in.atEnd()) {
				QString line = in.readLine();
				QStringList list = line.split(separator);
				if (!(parseToFloat(list[0], x) && parseToFloat(list[1], y) && parseToFloat(list[2], z)))
				{
					addStatus(QObject::tr("Invalid number in file: ") + line);
					return false;
				}
				cloud->push_back(PointType(x, y, z));
			}

			cloud->width = cloud->size();
			cloud->height = 1;
			cloud->is_dense = true;

			return true;
		}
		else
		{
			addStatus(QObject::tr("Can not open file."));
			return false;
		}
	}

	void ModelIO::writePCD(QString & fileURL, const QList<pcl::PointCloud<PointType>::Ptr> & clouds)
	{
		bool binaryMode = true;
		int cloudIndex = 1;
		foreach (pcl::PointCloud<PointType>::Ptr cloud, clouds)
		{
			QString url = fileURL;
			if (clouds.size() > 1)
			{
				url.insert(url.lastIndexOf("."), QStringLiteral("_") + QString::number(cloudIndex++));
			}
			pcl::io::savePCDFile<PointType>(url.toStdString(), *cloud, binaryMode);
		}
	}
}