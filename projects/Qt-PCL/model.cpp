#include "Model.h"

namespace railroad {

	void Model::resetClouds()
	{
		cloudsMap.clear();
	}

	QStringList Model::status() const
	{
		return statusList;
	}

	QStringList Model::clouds() const
	{
		return cloudsMap.keys();
	}

	int Model::cloudsIndex() const
	{
		return cloudsMapIndex;
	}

	void Model::setCloudsIndex(const int & index)
	{
		cloudsMapIndex = index;
	}

	const QString Model::INPUT_CLOUD("Input cloud");
	const QString Model::EUCLIDEAN_CLUSTERS("Euclidean Clusters");
}
