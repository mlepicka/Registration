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
	prop_RanSAC_max_iterations("rejection.ransac_iterations", 1000),
	pass_through("pass_through", false)
{
	registerProperty(prop_reject_correspondences);
	registerProperty(prop_max_distance);
	registerProperty(prop_use_RanSAC);
	registerProperty(prop_RanSAC_inliers_threshold);
	registerProperty(prop_RanSAC_max_iterations);
	registerProperty(pass_through);
}

CorrespondenceEstimation::~CorrespondenceEstimation() {
}

void CorrespondenceEstimation::prepareInterface() {
	// Register src-trg related data streams.
	registerStream("in_src_cloud_xyzsift", &in_src_cloud_xyzsift);
	registerStream("in_trg_cloud_xyzsift", &in_trg_cloud_xyzsift);
	registerStream("out_correspondences", &out_src_trg_correspondences);

	// Register lum related data streams.
	registerStream("in_lum_xyzsift", &in_lum_xyzsift);
	registerStream("out_lum_xyzsift", &out_lum_xyzsift);

	// Register models-scene related data streams.
	registerStream("in_scene_cloud_xyzsift", &in_trg_cloud_xyzsift);
	registerStream("in_model_clouds_xyzsift", &in_model_clouds_xyzsift);
	registerStream("out_models_scene_correspondences", &out_models_scene_correspondences);

	// Register src-trg correspondence estimation handler.
	registerHandler("estimateCorrespondencesForPairOfClouds", boost::bind(&CorrespondenceEstimation::estimateCorrespondencesForPairOfClouds, this));
	addDependency("estimateCorrespondencesForPairOfClouds", &in_src_cloud_xyzsift);
	addDependency("estimateCorrespondencesForPairOfClouds", &in_trg_cloud_xyzsift);

	// Register lum-related correspondence estimation handler.
	registerHandler("estimateCorrespondencesForLUMGraph", boost::bind(&CorrespondenceEstimation::estimateCorrespondencesForLUMGraph, this));
	addDependency("estimateCorrespondencesForLUMGraph", &in_lum_xyzsift);

	// Register src-trg correspondence estimation handler.
	registerHandler("estimateCorrespondencesBeteenModelsAndScene", boost::bind(&CorrespondenceEstimation::estimateCorrespondencesBeteenModelsAndScene, this));
	addDependency("estimateCorrespondencesBeteenModelsAndScene", &in_trg_cloud_xyzsift);
	addDependency("estimateCorrespondencesBeteenModelsAndScene", &in_model_clouds_xyzsift);

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


void CorrespondenceEstimation::estimateCorrespondencesForPairOfClouds() {
	CLOG(LTRACE) << "estimateCorrespondencesForPairOfClouds()";

	// Read clouds from input ports.
	pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud = in_src_cloud_xyzsift.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud = in_trg_cloud_xyzsift.read();


	// TODO if empty()
/*	// Remove NaNs.
	std::vector<int> indices;
	cloud_first->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_first, *cloud_first, indices);
	cloud_sec->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sec, *cloud_sec, indices);*/

	// Compute and return correspondences for input clouds.
	pcl::CorrespondencesPtr correspondences	= estimateCorrespondences(src_cloud, trg_cloud);
	out_src_trg_correspondences.write(correspondences);
}



void CorrespondenceEstimation::estimateCorrespondencesForLUMGraph() {
	CLOG(LTRACE) << "estimateCorrespondencesForLUMGraph()";

	// Temporary variables.
	pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud, trg_cloud;
	pcl::CorrespondencesPtr correspondences;

	// Read LUM graph from input port.
	pcl::registration::LUM<PointXYZSIFT>::Ptr lum_xyzsift = in_lum_xyzsift.read();

	// If pass_through - return the received graph.
	if (pass_through) {
		CLOG(LDEBUG) << "Passthrough - returning the received graph";
		out_lum_xyzsift.write(lum_xyzsift);
	}//: if passthrough


	// Iterate through graph and compute correspondences between all pairs of clouds.
	CLOG(LINFO) << "Computing correspondences between all pairs of clouds - this might take a while...";
	for (int i=0; i < lum_xyzsift->getNumVertices(); i++){

		// Get i-th cloud and transform it.
		pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_trans (new pcl::PointCloud<PointXYZSIFT>);
		trg_cloud = lum_xyzsift->getPointCloud(i);
		Eigen::Affine3f trg_tf  = lum_xyzsift->getTransformation(i);
		pcl::transformPointCloud(*trg_cloud, *trg_cloud_trans, trg_tf);

		for (int j=i+1; j < lum_xyzsift->getNumVertices(); j++){

			// Get j-th cloud and transform it.
			pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_trans (new pcl::PointCloud<PointXYZSIFT>);
			src_cloud = lum_xyzsift->getPointCloud(j);
			Eigen::Affine3f src_tf  = lum_xyzsift->getTransformation(j);
			pcl::transformPointCloud(*src_cloud, *src_cloud_trans, src_tf);

			// Compute correspondences.
			CLOG(LNOTICE) << "correspondences between: ("<<i<<","<<j<<")";
			correspondences = estimateCorrespondences(src_cloud_trans, trg_cloud_trans);
			// Set correspondences.
			lum_xyzsift->setCorrespondences(i, j, correspondences);
		}//: for j
	}//: for i

	// Return updated cloud.
	out_lum_xyzsift.write(lum_xyzsift);
}



void CorrespondenceEstimation::estimateCorrespondencesBeteenModelsAndScene() {
	CLOG(LTRACE) << "estimateCorrespondencesBeteenModelsAndScene()";

	// Temporary variables.
	pcl::PointCloud<PointXYZSIFT>::Ptr model_cloud;
	pcl::CorrespondencesPtr model_correspondences;
	std::vector<pcl::CorrespondencesPtr> all_correspondences;

	// Read clouds from input ports.
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud = in_trg_cloud_xyzsift.read();
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr>  model_clouds_xyzsift = in_model_clouds_xyzsift.read();

	// Iterate through models and find correspondences.
	CLOG(LINFO) << "Computing correspondences between scene and models - this might take a while...";
	for (int i = 0; i < model_clouds_xyzsift.size(); ++i) {
		// Get i-th model cloud.
		model_cloud = model_clouds_xyzsift[i];
		// Compute !MODEL 2 SCENE! correspondences.
		model_correspondences = estimateCorrespondences(model_cloud, scene_cloud);
		CLOG(LDEBUG) << "Found " << model_correspondences->size() << "between "<<i<<"-th model and scene";
		// Add to vector.
		all_correspondences.push_back(model_correspondences);
	}//: for


	out_models_scene_correspondences.write(all_correspondences);
}



pcl::CorrespondencesPtr CorrespondenceEstimation::estimateCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_, pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_) {
	CLOG(LTRACE) << "estimateCorrespondences(src,trg)";
	// Empty list of correspondences.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	// If pass_through - return empty list.
	if (pass_through) {
		CLOG(LDEBUG) << "Passthrough - returning no correspondences";
		return correspondences;
	}//: if passthrough

	// Create object responsible for correspondence estimation.
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
	// Set feature representation.
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation());
	correst.setPointRepresentation(point_representation);


	// Filter NaN points.
	pcl::PointCloud<PointXYZSIFT>::Ptr filtered_src_cloud(new pcl::PointCloud<PointXYZSIFT>);
	std::vector<int> src_indices;
	pcl::removeNaNFromPointCloud(*src_cloud_, *filtered_src_cloud, src_indices);
	//filtered_src_cloud->is_dense = false;

	pcl::PointCloud<PointXYZSIFT>::Ptr filtered_trg_cloud(new pcl::PointCloud<PointXYZSIFT>);
	std::vector<int> trg_indices;
	pcl::removeNaNFromPointCloud(*trg_cloud_, *filtered_trg_cloud, trg_indices);
	//filtered_trg_cloud->is_dense = false;


	// Set input clouds.
	correst.setInputSource(filtered_src_cloud);
	correst.setInputTarget(filtered_trg_cloud);

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
			crsac.setInputSource(filtered_src_cloud);
			crsac.setInputTarget(filtered_trg_cloud);
			crsac.setInlierThreshold(prop_RanSAC_inliers_threshold);
			crsac.setMaximumIterations(prop_RanSAC_max_iterations);
			crsac.setInputCorrespondences(correspondences);

			// Reject correspondences.
			crsac.getCorrespondences(*inliers);
			CLOG(LINFO) << "Correspondences remaining after RANSAC-based rejection: " << inliers->size();

			// Get computed transformation.
			Types::HomogMatrix hm = crsac.getBestTransformation() ;
			CLOG(LINFO) << "Found transformation:\n" << hm;

			// Return correspondences - inliers.
			return inliers;
		} else {
			// Use correspondence rejection distance.
			CLOG(LDEBUG) << "Using correspondence rejection based on Euclidean distance";
			// Use RANSAC to filter correspondences.
			pcl::CorrespondencesPtr inliers(new pcl::Correspondences()) ;

			pcl::registration:: CorrespondenceRejectorDistance cr ;
			cr.setInputSource<PointXYZSIFT>(filtered_src_cloud);
			cr.setInputTarget<PointXYZSIFT>(filtered_trg_cloud);
			cr.setMaximumDistance(prop_max_distance);
			cr.setInputCorrespondences(correspondences);

			// Reject correspondences.
			cr.getCorrespondences(*inliers);

			CLOG(LINFO) << "Correspondences remaining after Euclidean based rejection: " << inliers->size();
			// Return correspondences - inliers.
			return inliers;
		}//: else euclidean

	} else  {
		CLOG(LDEBUG) << "Returning all correspondences";
		// Return found correspondences.
		return correspondences;
	}//: if
}




} //: namespace CorrespondenceEstimation
} //: namespace Processors
