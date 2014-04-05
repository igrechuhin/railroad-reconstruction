#include "point_cloud_processor.h"

#include <QStringList>

namespace railroad {

	PointCloudProcessor::PointCloudProcessor(QObject * parent)
		: QObject(parent),
		  ModelIO(),
		  ModelViewer()
	{
	}

	void PointCloudProcessor::setValid(const bool & valid)
	{
		if (valid != cloudValid)
		{
			cloudValid = valid;
			emit validChanged();
		}
	}

	void PointCloudProcessor::addStatus(const QString & newStatus)
	{
		statusList.push_back(newStatus);
		emit statusChanged();
	}

	void PointCloudProcessor::readModel(const QUrl & url)
	{
		ModelIO::readModel(url);
		draw();
	}

}
