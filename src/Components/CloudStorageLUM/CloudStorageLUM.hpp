/*!
 * \file
 * \brief 
 * \author tkornuta
 */

#ifndef CloudStorageLUM_HPP_
#define CloudStorageLUM_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"
#include "Types/PointXYZSIFT.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// GraphSLAM structure.
#include <pcl/registration/lum.h>

namespace Processors {
namespace CloudStorageLUM {

/*!
 * \class CloudStorageLUM
 * \brief CloudStorageLUM processor class.
 *
 * CloudStorageLUM processor.
 */
class CloudStorageLUM: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudStorageLUM(const std::string & name = "CloudStorageLUM");

	/*!
	 * Destructor
	 */
	virtual ~CloudStorageLUM();

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

	/// Input stream containing XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzrgb;

	/// Input stream containing XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzsift;

	/// Input stream containing transformation between clouds.
	Base::DataStreamIn<Types::HomogMatrix, Base::DataStreamBuffer::Newest> in_transformation;

	/// Trigger - used for adding cloud to storage.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_add_cloud_trigger;

	/// Input data stream containing the SLAM graph which will be used.
	Base::DataStreamIn<pcl::registration::LUM<PointXYZSIFT>::Ptr> in_lum_xyzsift;


	// Output stream containing merged XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	// Output stream containing merged XYZSIFT cloud.
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Output stream: previous  XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_previous_cloud_xyzrgb;

	/// Output data stream containing the SLAM graph with estimated correspondences.
	Base::DataStreamOut<pcl::registration::LUM<PointXYZSIFT>::Ptr> out_lum_xyzsift;


	// Output stream: source  XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_src_cloud_xyzrgb;

	// Output stream: source  XYZSIFT cloud.
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_src_cloud_xyzsift;

	// Output stream: target  XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_trg_cloud_xyzrgb;

	// Output stream: target  XYZSIFT cloud.
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_trg_cloud_xyzsift;

	// Output stream: source-target  correspondences.
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_src_trg_correspondences;


	///  Property - store first cloud.
	Base::Property<bool> prop_store_first_cloud;

	///  Property - overwrite last cloud.
	Base::Property<bool> prop_overwrite_last_cloud;
	
	///  Property - returns previous cloud (n-1) or cloud merged from all stored (0-:n-1).
	Base::Property<bool> prop_return_previous_merged_cloud;


	///  Property - maximal number of stored clouds. The 0 value deactivates the limit. If reached, removes clouds from start of the list (resulting in a cyclic buffer).
	Base::Property<int> prop_clouds_limit;


	///  Property - index of source cloud.
	Base::Property<int> prop_src_cloud_index;

	///  Property - index of target cloud.
	Base::Property<int> prop_trg_cloud_index;


	/// Event handler function - adds point cloud to the storage.
	void onAddCloudButtonPressed();

	/// Event handler function - adds point cloud to the storage, externally triggered version.
	void onAddCloudTriggered();

	/// Adds point cloud to the storage.
	void add_cloud_to_storage();



	/// Event handler function - removes last point cloud from storage.
	void onRemoveLastCloudButtonPressed();

	/// Removes last point cloud from storage.
	void remove_last_cloud_to_storage();



	/// Event handler function - clears storage.
	void onClearStorageButtonPressed();

	/// Clears the storage.
	void clear_storage();


	/// Main storage management function.
	void update_storage();

	/// Publishes clouds merged from currently possesed vectors of clouds.
	void publish_merged_clouds();

    /// Return previous (single or merged) cloud.
    void return_previous_cloud();

	/// Event handler function - sets publishPreviousCloud_flag to true.
	void onPublishPreviousButtonPressed();

	/// Event handler function - store previous cloud, externally triggered version.
	void onReturnPreviousCloudTriggered();

	// Publishes previous (single or merged) cloud.
	void publishPreviousCloud();

	/// Event handler function - sets publishLumGraphSlam_flag to true.
	void onPublishLUMGraphSLAMButtonPressed();

	//  Publishes the current graph-SLAM structure.
	void publishLumGraphSlam();


	/// Event handler function - sets generateLumGraphSlam_flag to true.
	void  onGenerateLUMGraphSLAMButtonPressed();

	/// Generates the graph-SLAM structure basing on current clouds and transformations
	void  generateLumGraphSlam();


	/// Event handler function - updates the transfromation as well as the graph-SLAM structure basing on the input.
	void onUpdateTransfromationsBasingOnLumGraphSlam();


	/// Event handler function - sets publishSrcTrgCorrs_flag to true. 
	void onPublishSrcTrgCorrespondencesButtonPressed();

	/// Publishes source and target clouds along with correspondences.
	void publishSrcTrgCorrespondences();

	/// Event handler function - sets executeLUM_flag to true. 
	void onExecuteLUMButtonPressed();

	/// Runs LUM Graph SLAM optimization.
	void executeLUM();

private:
	/// Flag indicating whether the cloud should be added to storage.
	bool add_cloud_flag;

	/// Flag indicating that last cloud should be removed.
	bool remove_last_cloud_flag;

	/// Flag indicating that the storage should be cleared (remove all clouds).
	bool clear_storage_flag;

	//// Vector containing transformations - poses wrt to the first cloud. First transformation is an identity matrix.
	std::vector<Types::HomogMatrix> transformations;

	/// Vector containing XYZRGB clouds.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_xyzrgb;

	/// Vector containing XYZSIFT clouds.
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> clouds_xyzsift;

	/// Pointer to LUM graph-SLAM structure.
	pcl::registration::LUM<PointXYZSIFT>::Ptr lum_xyzsift;


	/// Flag indicating whether the one of the previous cloud (one or merged) should be returned as previous (i.e. to which the registration will be made in the next step).
	bool publishPreviousCloud_flag;

	/// Flag indicating that LUM graph SLAM structure should be published.
	bool publishLumGraphSlam_flag;

	/// Flag indicating that LUM graph SLAM structure should be generated.
	bool generateLumGraphSlam_flag;

	/// Flag indicating that pairs of source and target clouds along with correspondences should be published.
	bool publishSrcTrgCorrs_flag;

	/// Flag indicating that LUM optimization should be executed.
	bool executeLUM_flag;

};

} //: namespace CloudStorageLUM
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudStorageLUM", Processors::CloudStorageLUM::CloudStorageLUM)

#endif /* CloudStorageLUM_HPP_ */
