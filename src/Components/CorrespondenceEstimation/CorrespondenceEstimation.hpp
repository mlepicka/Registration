/*!
 * \file
 * \brief 
 * \author Tomek Kornuta, tkornuta@gmail.com
 */

#ifndef CORRESPONDENCEESTIMATION_HPP_
#define CORRESPONDENCEESTIMATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/correspondence.h>

#include <Types/HomogMatrix.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTFeatureRepresentation.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
// GraphSLAM structure.
#include <pcl/registration/lum.h>

namespace Processors {
namespace CorrespondenceEstimation {

/*!
 * \class CorrespondenceEstimation
 * \brief Class responsible for esimating the correspondences betwee two XYZSIFT point clouds.
 *
 * CorrespondenceEstimation processor.
 */
class CorrespondenceEstimation: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CorrespondenceEstimation(const std::string & name = "CorrespondenceEstimation");

	/*!
	 * Destructor
	 */
	virtual ~CorrespondenceEstimation();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	/// Input data stream containing source XYZSIFT cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_src_cloud_xyzsift;

	/// Input data stream containing target XYZSIFT cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_trg_cloud_xyzsift;

	/// Output data stream containing corespondences beetwen input clouds.
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_src_trg_correspondences;

	Base::DataStreamOut<pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT>::Ptr > out_corest;
	/// Input data stream containing vector of XYZSIFT clouds (objects/models).
	Base::DataStreamIn <std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_model_clouds_xyzsift;


	/// Output data stream containing vector of corespondences beetwen models and scene clouds.
	Base::DataStreamOut<std::vector<pcl::CorrespondencesPtr> > out_models_scene_correspondences;



	/// Input data stream containing the SLAM graph in which correspondences will be found.
	Base::DataStreamIn<pcl::registration::LUM<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_lum_xyzsift;

	/// Output data stream containing the SLAM graph with estimated correspondences.
	Base::DataStreamOut<pcl::registration::LUM<PointXYZSIFT>::Ptr> out_lum_xyzsift;


	/// Handler - estimates the correspondences for pair of input clouds.
	void estimateCorrespondencesForPairOfClouds();


	/// Handler - estimates the correspondences for whole LUM graph.
	void estimateCorrespondencesForLUMGraph();


	/// Handler - estimates the correspondences between models and scene.
	void estimateCorrespondencesBeteenModelsAndScene();


	/// Estimates the correspondences between source and targed point clouds.
	pcl::CorrespondencesPtr estimateCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_, pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_);

	/// Property - use correspondence rejection (RanSAC or simple euclidean distance metric). 
	Base::Property<bool> prop_reject_correspondences;

	/// Property - use RanSaC based correspondence rejection. 
	Base::Property<bool> prop_use_RanSAC;

	/// Property - the inliers threshold (set to 5cm, i.e. 0.05m).
	Base::Property<double> prop_RanSAC_inliers_threshold;

	/// Property - the maximum number of iterations (set to 1000). 
	Base::Property<double> prop_RanSAC_max_iterations;

	/// Property - the maximum allowed euclidean distance between correspondences.
	Base::Property<double> prop_max_distance;

	/// Property - return empty list of correspondences.
	Base::Property<bool> pass_through;

};

} //: namespace CorrespondenceEstimation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondenceEstimation", Processors::CorrespondenceEstimation::CorrespondenceEstimation)

#endif /* CORRESPONDENCEESTIMATION_HPP_ */
