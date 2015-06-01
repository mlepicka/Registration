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
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_src_cloud_xyzsift;

	/// Input data stream containing target XYZSIFT cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_trg_cloud_xyzsift;
	
	/// Output data stream containing corespondences beetwen input clouds.
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences;


	/// Handler - estimates the correspondences.
	void estimateCorrespondences();

	/// Property - use correspondence rejection (RanSAC or simple euclidean distance metric). 
	Base::Property<bool> prop_reject_correspondences;

	/// Property - use RanSaC based correspondence rejection. 
	Base::Property<bool> prop_use_RanSAC;

	/// Property - the inliers threshold (set to 5cm, i.e. 0.05m).
	Base::Property<float> prop_RanSAC_inliers_threshold;

	/// Property - the maximum number of iterations (set to 1000). 
	Base::Property<float> prop_RanSAC_max_iterations;

	/// Property - the maximum euclidean distance.
	Base::Property<float> prop_max_distance;

};

} //: namespace CorrespondenceEstimation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondenceEstimation", Processors::CorrespondenceEstimation::CorrespondenceEstimation)

#endif /* CORRESPONDENCEESTIMATION_HPP_ */
