#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H

#include "model_io.h"
#include "model_viewer.h"

#include <QObject>

namespace railroad {

	class PointCloudProcessor : public QObject, public ModelIO, public ModelViewer
	{
		Q_OBJECT

		Q_PROPERTY(bool valid READ valid WRITE setValid NOTIFY validChanged)
		Q_PROPERTY(QStringList status READ status NOTIFY statusChanged)
	public:
		explicit PointCloudProcessor(QObject * parent = 0);

		Q_INVOKABLE void readModel(const QUrl & url);

		Q_INVOKABLE void euclideanClusterExtraction();

		virtual void setValid(const bool & valid);
		virtual void addStatus(const QString & status);

	signals:
		void validChanged();
		void statusChanged();

	};

}

#endif // POINTCLOUDPROCESSOR_H
