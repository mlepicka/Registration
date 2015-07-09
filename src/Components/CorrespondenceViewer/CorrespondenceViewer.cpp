/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>
#include <sstream>
#include "CorrespondenceViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
//#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>

namespace Processors {
namespace CorrespondenceViewer {

CorrespondenceViewer::CorrespondenceViewer(const std::string & name) :
	Base::Component(name),
    prop_title("title", std::string("Correspondences")),
	prop_coordinate_system("coordinate_system",boost::bind(&CorrespondenceViewer::onCSShowClick, this, _2), true),
	prop_background_color("background_color", boost::bind(&CorrespondenceViewer::onBackgroundColorChange, this, _2), std::string("0,0,0")),
    src_cloud_xyzsift_point_size("source.cloud_xyzsift_point_size", 5),
    trg_cloud_xyzsift_point_size("target.cloud_xyzsift_point_size", 5),
    prop_clouds_colours("clouds_colours", std::string("255,0,0;255,0,0")),
    prop_correspondences_colours("correspondences_colours", std::string("255,0,0")),
    display_src_cloud_xyzrgb("source.display_cloud_xyzrgb", true),
    display_trg_cloud_xyzrgb("target.display_cloud_xyzrgb", true),
    display_src_cloud_xyzsift("source.display_cloud_xyzsift", true),
    display_trg_cloud_xyzsift("target.display_cloud_xyzsift", true),
    display_correspondences("correspondences.display_all", true),
    display_good_correspondences("correspondences.display_good", true),
	display_bounding_box("display_bounding_box", false),
    tx("translation.x", 0.1f),
    ty("translation.y", 0.1f),
    tz("translation.z", 0.1f),
	display_one_cluster("display_one_cluster", false),
	display_cluster("display_cluster", 0)
{
    registerProperty(prop_title);
	registerProperty(prop_coordinate_system);
	registerProperty(prop_background_color);

    registerProperty(src_cloud_xyzsift_point_size);
    registerProperty(trg_cloud_xyzsift_point_size);
    registerProperty(prop_clouds_colours);
    registerProperty(prop_correspondences_colours);
    registerProperty(display_src_cloud_xyzrgb);
    registerProperty(display_trg_cloud_xyzrgb);
    registerProperty(display_src_cloud_xyzsift);
    registerProperty(display_trg_cloud_xyzsift);
	registerProperty(display_correspondences);
	registerProperty(display_good_correspondences);
	registerProperty(display_bounding_box);
	registerProperty(tx);
	registerProperty(ty);
	registerProperty(tz);
	registerProperty(display_one_cluster);
	registerProperty(display_cluster);

	// Initialize viewer to NULL!
	viewer = NULL;
}

CorrespondenceViewer::~CorrespondenceViewer() {
}

void CorrespondenceViewer::onCSShowClick(const bool new_show_cs_){
    CLOG(LDEBUG) << "onCSShowClick show="<<new_show_cs_;

    if (!viewer)
    	return;

    if(new_show_cs_) {
	if (new_show_cs_) {
//#if PCL_VERSION_COMPARE(==,1,7,1)
//		viewer->addCoordinateSystem ();
		viewer->addCoordinateSystem (1.0,"reference", 0 );
//#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	} else {
//#if PCL_VERSION_COMPARE(>=,1,7,1)
//		viewer->removeCoordinateSystem ();
		viewer->removeCoordinateSystem ("reference", 0 );
		viewer->addCoordinateSystem (0.0,"reference", 0 );
//#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	}
    }

    prop_coordinate_system = new_show_cs_;
}

void CorrespondenceViewer::onBackgroundColorChange(std::string color_) {
    CLOG(LDEBUG) << "onBackgroundColorChange color=" << color_;
	try {
		// Parse string.
		vector<std::string> strs;
		boost::split(strs, color_, boost::is_any_of(","));
		if (strs.size() != 3)
			throw std::exception();

		// Try to cast to double and divide by 255.
		double r = boost::lexical_cast<double>(strs[0]) /255;
		double g = boost::lexical_cast<double>(strs[1]) /255;
		double b = boost::lexical_cast<double>(strs[2]) /255;

        CLOG(LINFO) << "onBackgroundColorChange r=" << r << " g=" << g << " b=" << b;
		// Change background color.
		if (viewer)
			viewer->setBackgroundColor(r, g, b);
	} catch (...) {
		CLOG(LWARNING)
                << "onBackgroundColorChange failed - invalid color format. Accepted format: r,g,b";
	}

}

void CorrespondenceViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_src_cloud_xyzsift", &in_src_cloud_xyzsift);
    registerStream("in_trg_cloud_xyzsift", &in_trg_cloud_xyzsift);
    registerStream("in_src_cloud_xyzrgb", &in_src_cloud_xyzrgb);
    registerStream("in_trg_cloud_xyzrgb", &in_trg_cloud_xyzrgb);
    registerStream("in_correspondences", &in_correspondences);
    registerStream("in_good_correspondences", &in_good_correspondences);
    registerStream("in_clustered_correspondences", &in_clustered_correspondences);

	// Register handlers
    registerHandler("on_clouds", boost::bind(&CorrespondenceViewer::on_clouds, this));
    addDependency("on_clouds", &in_src_cloud_xyzsift);
    addDependency("on_clouds", &in_trg_cloud_xyzsift);
    addDependency("on_clouds", &in_src_cloud_xyzrgb);
    //addDependency("on_clouds", &in_correspondences);
    addDependency("on_clouds", &in_trg_cloud_xyzrgb);

	// Register spin handler.
    registerHandler("on_spin", boost::bind(&CorrespondenceViewer::on_spin, this));
	addDependency("on_spin", NULL);
}

bool CorrespondenceViewer::onInit() {
    LOG(LTRACE) << "onInit";

    src_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    trg_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    src_cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    trg_cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    trg_cloud_xyzrgbtrans = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    trg_cloud_xyzsifttrans = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());

    correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences());
    good_correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences());

    viewer = new pcl::visualization::PCLVisualizer (prop_title);

	// Try to change background color.
	onBackgroundColorChange(prop_background_color);

	// Display/hide coordinate system.
	onCSShowClick(prop_coordinate_system);


	viewer->initCameraParameters ();
	//cloud_view_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    clusters = 0;

	return true;
}

bool CorrespondenceViewer::onFinish() {
	return true;
}

bool CorrespondenceViewer::onStop() {
	return true;
}

bool CorrespondenceViewer::onStart() {
	return true;
}

void CorrespondenceViewer::on_clouds() {
    LOG(LTRACE) << "on_clouds()";

    // Read clouds from ports.
    src_cloud_xyzrgb = in_src_cloud_xyzrgb.read();
    trg_cloud_xyzrgb = in_trg_cloud_xyzrgb.read();
    src_cloud_xyzsift = in_src_cloud_xyzsift.read();
    trg_cloud_xyzsift = in_trg_cloud_xyzsift.read();

    // Define translation between clouds.
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	//float tx = 0.3f, ty = 0.0f, tz = 0.0f ;
	trans(0, 3) = tx ; trans(1, 3) = ty ; trans(2, 3) = tz ;

    // Transform the target cloud.
    pcl::transformPointCloud(*trg_cloud_xyzrgb, *trg_cloud_xyzrgbtrans, trans) ;
    pcl::transformPointCloud(*trg_cloud_xyzsift, *trg_cloud_xyzsifttrans, trans) ;

    // Display clouds with corresponcences.
    displayXYZRGB();

    displayXYZSIFT();

    displayCorrespondences();
}


void CorrespondenceViewer::displayXYZRGB(){
    viewer->removePointCloud("viewcloud1") ;
    if(display_src_cloud_xyzrgb){
        std::vector<int> indices;
        src_cloud_xyzrgb->is_dense = false;
        pcl::removeNaNFromPointCloud(*src_cloud_xyzrgb, *src_cloud_xyzrgb, indices);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution1(src_cloud_xyzrgb);
        viewer->addPointCloud<pcl::PointXYZRGB>(src_cloud_xyzrgb, color_distribution1, "viewcloud1") ;
    }

    viewer->removePointCloud("viewcloud2") ;
    if(display_trg_cloud_xyzrgb){
        std::vector<int> indices;
        trg_cloud_xyzrgbtrans->is_dense = false;
        pcl::removeNaNFromPointCloud(*trg_cloud_xyzrgbtrans, *trg_cloud_xyzrgbtrans, indices);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution2(trg_cloud_xyzrgbtrans);
        viewer->addPointCloud<pcl::PointXYZRGB>(trg_cloud_xyzrgbtrans, color_distribution2, "viewcloud2") ;
    }

}


void CorrespondenceViewer::displayXYZSIFT(){
    // Retrieve SIFT colours.
    double srcr, srcg, srcb, trgr, trgg, trgb;

    std::string tmp_colours = prop_clouds_colours;
    vector<std::string> sift_colours;
    boost::split(sift_colours, tmp_colours, boost::is_any_of(";"));
    if (sift_colours.size() != 2) {
        CLOG(LWARNING)	<< "Invalid SIFT clouds color format. Setting default colour. Accepted format: r1,g1,b1;r2,g2,b2";
        srcr = 255;
        srcg = 0;
        srcb = 0;
        trgr = 255;
        trgg = 0;
        trgb = 0;
    } else {
        // Parse colour of src cloud.
        std::string colour = sift_colours[0];
        vector<std::string> strs;
        boost::split(strs, colour, boost::is_any_of(","));
        if (strs.size() != 3) {
            CLOG(LWARNING)	<< "Invalid SIFT source cloud color format. Setting default colour. Accepted format: r1,g1,b1;r2,g2,b2";
            srcr = 255;
            srcg = 0;
            srcb = 0;
        } else {
            // Try to cast to double and divide by 255.
            srcr = boost::lexical_cast<double>(strs[0]) /255;
            srcg = boost::lexical_cast<double>(strs[1]) /255;
            srcb = boost::lexical_cast<double>(strs[2]) /255;
        }//: else

        // Parse colour of trg cloud.
        colour = sift_colours[1];
        boost::split(strs, colour, boost::is_any_of(","));
        if (strs.size() != 3) {
            CLOG(LWARNING)	<< "Invalid SIFT target cloud color format. Setting default colour. Accepted format: r1,g1,b1;r2,g2,b2";
            trgr = 255;
            trgg = 0;
            trgb = 0;
        } else {
            // Try to cast to double and divide by 255.
            trgr = boost::lexical_cast<double>(strs[0]) /255;
            trgg = boost::lexical_cast<double>(strs[1]) /255;
            trgb = boost::lexical_cast<double>(strs[2]) /255;
        }//: else


    }//: else


    viewer->removePointCloud("siftcloud1") ;
    if(display_src_cloud_xyzsift){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*src_cloud_xyzsift,*cloud_xyz1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz1, "siftcloud1") ;
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, src_cloud_xyzsift_point_size, "siftcloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, srcr, srcg, srcb, "siftcloud1");
    }

    viewer->removePointCloud("siftcloud2") ;
    if(display_trg_cloud_xyzsift){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*trg_cloud_xyzsifttrans,*cloud_xyz2);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz2, "siftcloud2") ;
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, trg_cloud_xyzsift_point_size, "siftcloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, trgr, trgg, trgb, "siftcloud2");
    }
}


void CorrespondenceViewer::displayCorrespondences(){
    CLOG(LTRACE) << "displayCorrespondences";

    /// Cleanup.
    if(in_clustered_correspondences.empty()){
        clustered_corrs.clear();
    } else {
        clustered_corrs = in_clustered_correspondences.read();
    }//: else

    // Remove clusters of correspondences.
    for(int i = 0; i < clusters; i++){
        ostringstream ss;
        ss << i;
        string str = ss.str();
        viewer->removeCorrespondences(std::string("correspondences")+str) ;
    }//: else
    clusters = clustered_corrs.size();

    // Remove bounding boxes.
    viewer->removeAllShapes();


    /// Parse colour.
    double r,g,b;
    vector<std::string> strs;
    std::string tmp_colour = prop_correspondences_colours;
    boost::split(strs, tmp_colour, boost::is_any_of(","));
    if (strs.size() != 3) {
        CLOG(LWARNING)	<< "Invalid corresponcendes color format. Setting default colour. Accepted format: r,g,b";
        r = 255;
        g = 0;
        b = 0;
    }

    // Try to cast to double and divide by 255.
    r = boost::lexical_cast<double>(strs[0]) /255;
    g = boost::lexical_cast<double>(strs[1]) /255;
    b = boost::lexical_cast<double>(strs[2]) /255;


    /// Display correspondences.
    // If no clustered corrs display corrs from in_correspondences and in_good_correspondences.
    if(clusters == 0){
        CLOG(LTRACE) << "Displaying all correspondences";
        // Display correspondences.
        if(!in_correspondences.empty())
            correspondences = in_correspondences.read();
        viewer->removeCorrespondences("correspondences");
        if(display_correspondences){

            viewer->addCorrespondences<PointXYZSIFT>(src_cloud_xyzsift, trg_cloud_xyzsifttrans, *correspondences, "correspondences") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "correspondences") ;
        }
        // Display good correspondences.
        if(!in_good_correspondences.empty())
            good_correspondences = in_good_correspondences.read();
        viewer->removeCorrespondences("good_correspondences");
        if (display_good_correspondences){
            viewer->addCorrespondences<PointXYZSIFT>(src_cloud_xyzsift, trg_cloud_xyzsifttrans, *good_correspondences, "good_correspondences") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "good_correspondences") ;
        }
    }//: if

    //Display clustered corrs
    else if(display_correspondences){
        CLOG(LTRACE) << "Displaying clusters of correspondences";
        viewer->removeCorrespondences("correspondences");
        viewer->removeCorrespondences("good_correspondences");
        //Display only one choosen cluster
        if(display_one_cluster){
            int display_cluster_ = display_cluster;
            if(display_cluster >= clusters){
                CLOG(LTRACE) << "Less than "<< display_cluster+1 << " clusters! Displaying cluster 0";
                display_cluster_ = 0;
            }
            viewer->addCorrespondences<PointXYZSIFT>(trg_cloud_xyzsifttrans, src_cloud_xyzsift, clustered_corrs[display_cluster_], "correspondences0") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "correspondences0") ;
            //Display Bounding Box
            if(display_bounding_box){
                CLOG(LTRACE) << "CorrespondenceViewer Display Bounding Box";
                vector<int> indices;
                for(int i = 0; i < clustered_corrs[display_cluster_].size(); i++){
                    indices.push_back(clustered_corrs[display_cluster_][i].index_match);
                }
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*src_cloud_xyzsift, indices, min_pt, max_pt);
                viewer->addCube (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], 255, 255, 255);

            }
        }
        //Display all clusters
        else{
            for(int i = 0; i< clustered_corrs.size(); i++){
            	// Random colors for given cluster.
                int c[3], index, r,g,b;
            	index = rand()%3;
                c[index+i] = 255;
                c[(index+1+i)%3] = 0;
                c[(index+2+i)%3] = 100 + rand()%155;
                r=c[0];g=c[1];b=c[2];
                //std::cout<<r<<","<<g<<","<<b<<endl;

                ostringstream ss;
                ss << i;
                string str = ss.str();
                viewer->addCorrespondences<PointXYZSIFT>(trg_cloud_xyzsifttrans, src_cloud_xyzsift, clustered_corrs[i], "correspondences"+str) ;
                viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "correspondences"+str) ;
                //Display Bounding Box
                if(display_bounding_box){
                    CLOG(LTRACE) << "CorrespondenceViewer Display Bounding Box";
                    vector<int> indices;
                    for(int j = 0; j < clustered_corrs[i].size(); j++){
                        indices.push_back(clustered_corrs[i][j].index_match);
                    }
                    Eigen::Vector4f min_pt, max_pt;
                    pcl::getMinMax3D(*src_cloud_xyzsift, indices, min_pt, max_pt);
                    viewer->addCube (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], r, g, b, "cube"+str);

                }
            }
        }
    }
}


void CorrespondenceViewer::on_spin() {
	viewer->spinOnce (100);
}


} //: namespace CorrespondenceViewer
} //: namespace Processors
