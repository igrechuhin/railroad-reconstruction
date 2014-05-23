#include "point_cloud_processor.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <QStringList>

namespace railroad {

	PointCloudProcessor::PointCloudProcessor(QObject * parent)
		: QObject(parent),
		  ModelIO(),
		  ModelViewer()
	{
	}

	void PointCloudProcessor::addStatus(const QString & newStatus)
	{
		statusList.push_back(newStatus);
		emit statusChanged();
	}

	void PointCloudProcessor::addClouds(const QString & name, const QList<pcl::PointCloud<PointType>::Ptr>& clouds)
	{
		cloudsMap.insert(name, clouds);
		emit cloudsChanged();
	}

	void PointCloudProcessor::readModel(const QUrl & url)
	{
		if (ModelIO::readModel(url))
		{
			setCloudsIndex(clouds().indexOf(INPUT_CLOUD));
			ModelViewer::draw(cloudsMap[INPUT_CLOUD]);
		}
	}

	void PointCloudProcessor::writeModel(const QUrl & url, const QString & cloudName)
	{
		ModelIO::writeModel(url, cloudName);
	}

	Q_INVOKABLE void PointCloudProcessor::euclideanClusterExtraction(const QString & cloudName)
	{
		auto cloud = cloudsMap[cloudName];
		_doEuclideanClusterExtraction(cloud);
		setCloudsIndex(clouds().indexOf(EUCLIDEAN_CLUSTERS));
		ModelViewer::draw(cloudsMap[EUCLIDEAN_CLUSTERS]);
	}

	void PointCloudProcessor::_doEuclideanClusterExtraction(const QList<pcl::PointCloud<PointType>::Ptr> & clouds)
	{
		QList<pcl::PointCloud<PointType>::Ptr> clusteredClouds;
		foreach (pcl::PointCloud<PointType>::Ptr cloud, clouds)
		{
			// Create the filtering object: downsample the dataset using a leaf size of 10cm
			pcl::PointCloud<PointType>::Ptr cloudFiltered (new pcl::PointCloud<PointType> ());
			pcl::VoxelGrid<PointType> vg;
			vg.setInputCloud (cloud);
			vg.setLeafSize (0.1f, 0.1f, 0.1f);
			vg.filter (*cloudFiltered);
			int nr_points = static_cast<int>(cloudFiltered->points.size());
			addStatus(QStringLiteral("PointCloud after filtering has: ") + QString::number(nr_points) + QStringLiteral(" data points."));

			// Create the segmentation object for the planar model and set all the parameters
			pcl::SACSegmentation<PointType> seg;
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.02);

			pcl::PointCloud<PointType>::Ptr cloud_f (new pcl::PointCloud<PointType>);
			while (cloudFiltered->points.size () > 0.3 * nr_points)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud (cloudFiltered);
				seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0)
				{
					addStatus(QStringLiteral("Could not estimate a planar model for the given dataset."));
					break;
				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<PointType> extract;
				extract.setInputCloud (cloudFiltered);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Get the points associated with the planar surface
				extract.filter (*cloud_plane);
				addStatus(QStringLiteral("PointCloud representing the planar component: ") + QString::number(cloud_plane->points.size()) + QStringLiteral(" data points."));

				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_f);
				*cloudFiltered = *cloud_f;
			}

			// Creating the KdTree object for the search method of the extraction
			pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
			tree->setInputCloud (cloudFiltered);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointType> ec;
			ec.setClusterTolerance (0.2); // 20cm
			ec.setMinClusterSize (100);
			ec.setMaxClusterSize (250000);
			ec.setSearchMethod (tree);
			ec.setInputCloud (cloudFiltered);
			ec.extract (cluster_indices);

			for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
				for (auto pit = it->indices.begin (); pit != it->indices.end (); pit++)
				{
					cloud_cluster->points.push_back (cloudFiltered->points[*pit]); //*
				}
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				clusteredClouds.push_back(cloud_cluster);
			}
		}
		addClouds(EUCLIDEAN_CLUSTERS, clusteredClouds);
	}

	void PointCloudProcessor::draw(const QString & cloudName)
	{
		setCloudsIndex(clouds().indexOf(cloudName));
		ModelViewer::draw(cloudsMap[cloudName]);
	}
}
