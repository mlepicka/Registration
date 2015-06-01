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
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_source_xyzsift;

	/// Input data stream containing target XYZSIFT cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_target_xyzsift;
	
	/// Output data stream containing corespondences beetwen input clouds.
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences;


	/// Handler - estimates the correspondences.
	void estimateCorrespondences();


};

} //: namespace CorrespondenceEstimation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondenceEstimation", Processors::CorrespondenceEstimation::CorrespondenceEstimation)

#endif /* CORRESPONDENCEESTIMATION_HPP_ */
