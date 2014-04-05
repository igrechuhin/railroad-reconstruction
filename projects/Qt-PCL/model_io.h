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
		void readModel(const QUrl & url);

	private:
		bool readPCD(QString & fileURL);
		bool readCSV(QString & fileURL);	
	};

}
#endif // MODEL_IO_H
