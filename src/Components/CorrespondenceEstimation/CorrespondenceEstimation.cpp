/*!
 * \file
 * \brief
 * \author Tomek Kornuta, tkornuta@gmail.com
 */

#include <memory>
#include <string>

#include "CorrespondenceEstimation.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_rejection_sample_consensus.h>


namespace Processors {
namespace CorrespondenceEstimation {

CorrespondenceEstimation::CorrespondenceEstimation(const std::string & name) :
		Base::Component(name)  {

}

CorrespondenceEstimation::~CorrespondenceEstimation() {
}

void CorrespondenceEstimation::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_source_xyzsift", &in_cloud_source_xyzsift);
	registerStream("in_cloud_target_xyzsift", &in_cloud_target_xyzsift);
	registerStream("out_correspondences", &out_correspondences);

	// Register handlers.
	registerHandler("estimateCorrespondences", boost::bind(&CorrespondenceEstimation::estimateCorrespondences, this));
	addDependency("estimateCorrespondences", &in_cloud_source_xyzsift);
	addDependency("estimateCorrespondences", &in_cloud_target_xyzsift);

}

bool CorrespondenceEstimation::onInit() {
	CLOG(LTRACE) << "onInit()";

	return true;
}

bool CorrespondenceEstimation::onFinish() {
	CLOG(LTRACE) << "onFinish()";
	return true;
}

bool CorrespondenceEstimation::onStop() {
	CLOG(LTRACE) << "onStop()";
	return true;
}

bool CorrespondenceEstimation::onStart() {
	CLOG(LTRACE) << "onStart()";
	return true;
}


void CorrespondenceEstimation::estimateCorrespondences() {
	CLOG(LTRACE) << "estimateCorrespondences()";

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_src = in_cloud_source_xyzsift.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_tgt = in_cloud_target_xyzsift.read();

	SIFTFeatureRepresentation sift;
	// TODO if empty()

/*	// Remove NaNs.
	std::vector<int> indices;
	cloud_first->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_first, *cloud_first, indices);
	cloud_sec->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sec, *cloud_sec, indices);*/

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;

	// Set feature representation.
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation());
	correst.setPointRepresentation(point_representation);

	// Set input clouds.
	correst.setInputSource(cloud_src);
	correst.setInputTarget(cloud_tgt);

	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences);

	CLOG(LINFO) << "Found correspondences: " << correspondences->size();

	// Return found correspondences.
	out_correspondences.write(correspondences);

/*
	pcl::CorrespondencesPtr inliers(new pcl::Correspondences()) ;
	Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_first, cloud_sec, correspondences, *inliers, properties);
*/

}


} //: namespace CorrespondenceEstimation
} //: namespace Processors
