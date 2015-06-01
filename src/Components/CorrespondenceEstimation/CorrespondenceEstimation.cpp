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
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>

namespace Processors {
namespace CorrespondenceEstimation {

CorrespondenceEstimation::CorrespondenceEstimation(const std::string & name) :
	Base::Component(name),
	prop_reject_correspondences("rejection.use",false),
	prop_use_RanSAC("rejection.use_sample_consensus",false),
	prop_max_distance("rejection.max_distance", 0.005f),
	prop_RanSAC_inliers_threshold("rejection.ransac_inliers_threshold", 0.05f),
	prop_RanSAC_max_iterations("rejection.ransac_iterations", 1000)
{
	registerProperty(prop_reject_correspondences);
	registerProperty(prop_max_distance);
	registerProperty(prop_use_RanSAC);
	registerProperty(prop_RanSAC_inliers_threshold);
	registerProperty(prop_RanSAC_max_iterations);
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
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_trg = in_cloud_target_xyzsift.read();


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
	correst.setInputTarget(cloud_trg);

	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences);
	CLOG(LINFO) << "Found correspondences: " << correspondences->size();

	// Correspondence rejection.
	if (prop_reject_correspondences) {
		CLOG(LDEBUG) << "Correspondence rejection";
		if (prop_use_RanSAC) {
			CLOG(LDEBUG) << "Using RanSAC-based correspondence rejection";
			// Use RANSAC to filter correspondences.
			pcl::CorrespondencesPtr inliers(new pcl::Correspondences()) ;

			pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> crsac ;
			crsac.setInputSource(cloud_src);
			crsac.setInputTarget(cloud_trg);
			crsac.setInlierThreshold(prop_RanSAC_inliers_threshold);
			crsac.setMaximumIterations(prop_RanSAC_max_iterations);
			crsac.setInputCorrespondences(correspondences);

			// Reject correspondences.
			crsac.getCorrespondences(*inliers);
			CLOG(LINFO) << "Correspondences remaining after RANSAC-based rejection: " << inliers->size();
			// Return inliers.
			out_correspondences.write(inliers);

			// Get computed transformation.
			Types::HomogMatrix hm = crsac.getBestTransformation() ;

			CLOG(LINFO) << "Found transformation:\n" << hm;
		} else {
			// Use correspondence rejection distance.
			CLOG(LDEBUG) << "Using correspondence rejection based on Euclidean distance";
			// Use RANSAC to filter correspondences.
			pcl::CorrespondencesPtr inliers(new pcl::Correspondences()) ;

			pcl::registration:: CorrespondenceRejectorDistance cr ;
			cr.setInputSource<PointXYZSIFT>(cloud_src);
			cr.setInputTarget<PointXYZSIFT>(cloud_trg);
			cr.setMaximumDistance(prop_max_distance);
			cr.setInputCorrespondences(correspondences);

			// Reject correspondences.
			cr.getCorrespondences(*inliers);

			CLOG(LINFO) << "Correspondences remaining after Euclidean based rejection: " << inliers->size();
			// Return inliers.
			out_correspondences.write(inliers);
		}//: else euclidean

	} else  {
		CLOG(LDEBUG) << "Returning all correspondences";
		// Return found correspondences.
		out_correspondences.write(correspondences);
	}//: if

}


} //: namespace CorrespondenceEstimation
} //: namespace Processors
