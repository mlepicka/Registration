/*!
 * \file
 * \brief 
 * \author tkornuta
 */

#ifndef PairwiseRegistration_HPP_
#define PairwiseRegistration_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace PairwiseRegistration {

/*!
 * \class PairwiseRegistration
 * \brief PairwiseRegistration processor class.
 *
 * PairwiseRegistration processor.
 */
class PairwiseRegistration: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PairwiseRegistration(const std::string & name = "PairwiseRegistration");

	/*!
	 * Destructor
	 */
	virtual ~PairwiseRegistration();

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


	/// Input data stream containing XYZ cloud.
//	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyz;

	/// Input data stream containing source (previous) XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest> in_src_cloud_xyzrgb;

	/// Input data stream containing target (next) XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest> in_trg_cloud_xyzrgb;

	/// Resulting transformation between XYZ clouds.
	Base::DataStreamOut <Types::HomogMatrix> out_transformation_xyz;

	/// Resulting transformation between XYZRGB clouds.
	Base::DataStreamOut <Types::HomogMatrix> out_transformation_xyzrgb;

	/// Source (previous) cloud, to which the component will try to align.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_xyzrgb;


	/****************** ICP PROPERTIES ***********************/

	///  Property - use ICP.
	Base::Property<bool> prop_ICP;

	///  Property - ICP condition: the max correspondence distance (correspondences with higher distances will be ignored).
	Base::Property<float> prop_ICP_MaxCorrespondenceDistance;

	///  Property - ICP condition: maximum number of iterations.
	Base::Property<int> prop_ICP_MaximumIterations;

	///  Property - ICP condition: the epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value.
	Base::Property<double> prop_ICP_TransformationEpsilon;

	///  Property - ICP condition: the sum of Euclidean squared errors is smaller than a user defined threshold.
	Base::Property<float> prop_ICP_EuclideanFitnessEpsilon;

	///  Property - use colour in ICP.
	Base::Property<bool> prop_ICP_colour;

	///  Property - use normals in ICP.
	Base::Property<bool> prop_ICP_normals;


	/// Align XYZ clouds - handler.
//	void pairwise_registration_xyz();

	/// Aligns XYZRGB clouds - handler.
	void pairwise_registration_xyzrgb();

	// Performs ICP-based pairwise registration.
	Types::HomogMatrix pairwise_icp_based_registration_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_xyzrgb_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb_);
};

} //: namespace PairwiseRegistration
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PairwiseRegistration", Processors::PairwiseRegistration::PairwiseRegistration)

#endif /* PairwiseRegistration_HPP_ */
