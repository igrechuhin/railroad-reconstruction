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

	void ModelIO::readModel(const QUrl & url)
	{
		QString fileURL(url.toLocalFile());
		addStatus(QObject::tr("Reading file: ") + fileURL);
		if (url.isValid())
		{
			resetCloud();
			QString fileType = QFileInfo(fileURL).suffix();
			if (fileType == QStringLiteral("pcd"))
			{
				setValid(readPCD(fileURL));
			}
			else if (fileType == QStringLiteral("csv"))
			{
				setValid(readCSV(fileURL));
			}

			addStatus((valid() ? QObject::tr("File read: ") : QObject::tr("Couldn't read file: ")) + fileURL);
		}
		else
		{
			addStatus(QObject::tr("Invalid file url: ") + fileURL);
		}	
	}

	bool ModelIO::readPCD(QString & fileURL)
	{
		return pcl::io::loadPCDFile<PointType> (fileURL.toStdString(), *cloud) != -1;
	}

	bool ModelIO::readCSV(QString & fileURL)
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

}