/*!
 * \file
 * \brief 
 * \author tkornuta
 */

#ifndef CloudStorage_HPP_
#define CloudStorage_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"
#include "Types/PointXYZSIFT.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace CloudStorage {

/*!
 * \class CloudStorage
 * \brief CloudStorage processor class.
 *
 * CloudStorage processor.
 */
class CloudStorage: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudStorage(const std::string & name = "CloudStorage");

	/*!
	 * Destructor
	 */
	virtual ~CloudStorage();

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

	/// Input stream containing XYZ cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyz;

	/// Input stream containing XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzrgb;

	/// Input stream containing XYZRGB cloud.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzsift;

	/// Input stream containing transformation between clouds.
	Base::DataStreamIn<Types::HomogMatrix, Base::DataStreamBuffer::Newest> in_transformation;

	/// Trigger - used for adding cloud to storage.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_add_cloud_trigger;

	// Output stream containing merged XYZ cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;

	// Output stream containing merged XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	// Output stream containing merged XYZSIFT cloud.
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Output stream: previous  XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_previous_cloud_xyzrgb;

	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_previous_cloud_xyzsift;
	///  Property - store first cloud.
	Base::Property<bool> prop_store_first_cloud;

	///  Property - overwrite last cloud.
	Base::Property<bool> prop_overwrite_last_cloud;
	
	///  Property - returns previous cloud (n-1) or cloud merged from all stored (0-:n-1).
	Base::Property<bool> prop_return_previous_merged_cloud;


	///  Property - maximal number of stored clouds. The 0 value deactivates the limit. If reached, removes clouds from start of the list (resulting in a cyclic buffer).
	Base::Property<int> prop_clouds_limit;



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


private:
	/// Flag indicating whether the cloud should be added to storage.
	bool add_cloud_flag;

	/// Flag indicating that last cloud should be removed.
	bool remove_last_cloud_flag;

	/// Flag indicating that the storage should be cleared (remove all clouds).
	bool clear_storage_flag;

	// Vector containing transformations - poses wrt to the first cloud. First transformation is an identity matrix.
	std::vector<Types::HomogMatrix> transformations;

	// Vector containing XYZRGB clouds.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_xyzrgb;

	// Vector containing XYZ clouds.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_xyz;

	// Vector containing XYZSIFT clouds.
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> clouds_xyzsift;

};

} //: namespace CloudStorage
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudStorage", Processors::CloudStorage::CloudStorage)

#endif /* CloudStorage_HPP_ */
