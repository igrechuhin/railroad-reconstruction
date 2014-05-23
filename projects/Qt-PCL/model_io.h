#ifndef MODEL_IO_H
#define MODEL_IO_H

#include "model.h"

#include <QUrl>

namespace railroad {

	class ModelIO : virtual public Model
	{
	public:
		ModelIO();
	
	protected:
		bool readModel(const QUrl & url);
		void writeModel(const QUrl & url, const QString & cloudName);

	private:
		bool readPCD(QString & fileURL, pcl::PointCloud<PointType>::Ptr cloud);
		bool readCSV(QString & fileURL, pcl::PointCloud<PointType>::Ptr cloud);

		void writePCD(QString & fileURL, const QList<pcl::PointCloud<PointType>::Ptr>& clouds);
	};

}
#endif // MODEL_IO_H
