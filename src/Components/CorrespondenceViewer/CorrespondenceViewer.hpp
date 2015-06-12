/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CorrespondenceViewer_HPP_
#define CorrespondenceViewer_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <Types/PointXYZSIFT.hpp> 

#include <Types/MatrixTranslator.hpp>
#include <opencv2/core/core.hpp>


//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

//#include <pcl/visualization/point_cloud_geometry_handlers.h>

namespace Processors {
namespace CorrespondenceViewer {

/*!
 * \class CorrespondenceViewer
 * \brief CorrespondenceViewer processor class.
 *
 * CorrespondenceViewer processor.
 */
class CorrespondenceViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CorrespondenceViewer(const std::string & name = "CorrespondenceViewer");

	/*!
	 * Destructor
	 */
	virtual ~CorrespondenceViewer();

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

	/// PCL Viewer.
	pcl::visualization::PCLVisualizer * viewer;

	// Input data streams

    Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_src_cloud_xyzsift;
    Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_trg_cloud_xyzsift;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_src_cloud_xyzrgb;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_trg_cloud_xyzrgb;
	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondences;
	Base::DataStreamIn<pcl::CorrespondencesPtr> in_good_correspondences;
	Base::DataStreamIn<std::vector<pcl::Correspondences> > in_clustered_correspondences;
	// Output data streams

	// Handlers
	Base::EventHandler2 h_on_clouds;
	Base::EventHandler2 h_on_spin;

	// Handlers
	void on_clouds();
	void on_spin();

	/// Handler for showing/hiding coordinate system.
	void onCSShowClick(bool new_show_cs_);

	/// Handler for changing background color.
	void onBackgroundColorChange(std::string color_);

    /// Property - size of point from source cloud.
    Base::Property<int> src_cloud_xyzsift_point_size;

    /// Property - size of point from target cloud.
    Base::Property<int> trg_cloud_xyzsift_point_size;

	/// Property for setting the colours of clouds. From default it will be set to 2 rows with 255, 0, 0 (red).
    Base::Property<std::string> prop_clouds_colours;

	/// Property for setting the colour of correspondences. From default it will be set to 1 row with 255, 0, 0 (red).
    Base::Property<std::string> prop_correspondences_colours;

    Base::Property<bool> display_src_cloud_xyzrgb;
    Base::Property<bool> display_trg_cloud_xyzrgb;
    Base::Property<bool> display_src_cloud_xyzsift;
    Base::Property<bool> display_trg_cloud_xyzsift;
	Base::Property<bool> display_correspondences;
	Base::Property<bool> display_good_correspondences;
	Base::Property<bool> display_bounding_box;


	Base::Property<float> tx;
	Base::Property<float> ty;
	Base::Property<float> tz;

	Base::Property<bool> display_one_cluster;
	Base::Property<int> display_cluster;

	int clusters;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_xyzrgb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb;
    pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_xyzsift;
    pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsift;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgbtrans;
    pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsifttrans;

	pcl::CorrespondencesPtr correspondences;
	pcl::CorrespondencesPtr good_correspondences;

	std::vector<pcl::Correspondences> clustered_corrs;

	/// Property: name of the window.
    Base::Property<std::string> prop_title;

	/// Property: display/hide coordinate system.
	Base::Property<bool> prop_coordinate_system;

	/// Property: background color. As default it is set to 1 row with 0, 0, 0 (black).
	Base::Property<std::string> prop_background_color;

private:

    // Displays and refreshes XYZRGB clouds.
    void displayXYZRGB();

    // Displays and refreshes XYZSIFT clouds.
    void displayXYZSIFT();

    // Displays and refreshes correspondences.
    void displayCorrespondences();

};

} //: namespace CorrespondenceViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondenceViewer", Processors::CorrespondenceViewer::CorrespondenceViewer)

#endif /* CorrespondenceViewer_HPP_ */
