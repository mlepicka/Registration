/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "CloudStorageLUM.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace CloudStorageLUM {

CloudStorageLUM::CloudStorageLUM(const std::string & name) :
	Base::Component(name),
	prop_store_first_cloud("StoreFirstCloud",true),
	prop_overwrite_last_cloud("OverwriteLastCloud",false),
	prop_return_previous_merged_cloud("ReturnPreviousMergedCloud",false),
	prop_clouds_limit("CloudsLimit",0),
	prop_src_cloud_index("src_cloud_index",0),
	prop_trg_cloud_index("trg_cloud_index",0)
{
	// Register properties.
	registerProperty(prop_store_first_cloud);
	registerProperty(prop_overwrite_last_cloud);
	registerProperty(prop_return_previous_merged_cloud);

	registerProperty(prop_clouds_limit);
	prop_clouds_limit.addConstraint("0");
	prop_clouds_limit.addConstraint("999");

	registerProperty(prop_src_cloud_index);
	prop_src_cloud_index.addConstraint("0");
	prop_src_cloud_index.addConstraint("999");

	registerProperty(prop_trg_cloud_index);
	prop_trg_cloud_index.addConstraint("0");
	prop_trg_cloud_index.addConstraint("999");

}

CloudStorageLUM::~CloudStorageLUM() {
}

void CloudStorageLUM::prepareInterface() {
	// Register input data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_transformation", &in_transformation);
	registerStream("in_add_cloud_trigger", &in_add_cloud_trigger);
	registerStream("in_lum_xyzsift", &in_lum_xyzsift);
	registerStream("in_publish_previous_cloud_trigger", &in_publish_previous_cloud_trigger);


	// Register output data streams.
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_previous_cloud_xyzrgb", &out_previous_cloud_xyzrgb);
	registerStream("out_lum_xyzsift", &out_lum_xyzsift);
	registerStream("out_src_cloud_xyzrgb", &out_src_cloud_xyzrgb);
	registerStream("out_src_cloud_xyzsift", &out_src_cloud_xyzsift);
	registerStream("out_trg_cloud_xyzrgb", &out_trg_cloud_xyzrgb);
	registerStream("out_trg_cloud_xyzsift", &out_trg_cloud_xyzsift);
	registerStream("out_src_trg_correspondences", &out_src_trg_correspondences);


	// Register button-triggered handlers.
	registerHandler("Add cloud", boost::bind(&CloudStorageLUM::onAddCloudButtonPressed, this));
	registerHandler("Remove last cloud", boost::bind(&CloudStorageLUM::onRemoveLastCloudButtonPressed, this));
	registerHandler("Clear storage", boost::bind(&CloudStorageLUM::onClearStorageButtonPressed, this));
	registerHandler("Publish previous cloud", boost::bind(&CloudStorageLUM::onPublishPreviousButtonPressed, this));

	// Register externally-triggered handler.
	registerHandler("onAddCloudTriggered", boost::bind(&CloudStorageLUM::onAddCloudTriggered, this));
	addDependency("onAddCloudTriggered", &in_add_cloud_trigger);

	registerHandler("onReturnPreviousCloudTriggered", boost::bind(&CloudStorageLUM::onReturnPreviousCloudTriggered, this));
	addDependency("onReturnPreviousCloudTriggered", &in_publish_previous_cloud_trigger);

	registerHandler("LUM publish graph", boost::bind(&CloudStorageLUM::onPublishLUMGraphSLAMButtonPressed, this));

	registerHandler("LUM generate graph", boost::bind(&CloudStorageLUM::onGenerateLUMGraphSLAMButtonPressed, this));

	registerHandler("onUpdateTransfromationsBasingOnLumGraphSlam", boost::bind(&CloudStorageLUM::onUpdateTransfromationsBasingOnLumGraphSlam, this));
	addDependency("onUpdateTransfromationsBasingOnLumGraphSlam", &in_lum_xyzsift);

	registerHandler("Publish src-trg correspondences", boost::bind(&CloudStorageLUM::onPublishSrcTrgCorrespondencesButtonPressed, this));

	registerHandler("LUM execute optimization", boost::bind(&CloudStorageLUM::onExecuteLUMButtonPressed, this));

	// Register "main" storage management method.
	registerHandler("update_storage", boost::bind(&CloudStorageLUM::update_storage, this));
	addDependency("update_storage", NULL);


}

bool CloudStorageLUM::onInit() {
	// Initialize flags.
	remove_last_cloud_flag = false;
	clear_storage_flag = false;
	publishPreviousCloud_flag = false;
	publishLumGraphSlam_flag = false;
	generateLumGraphSlam_flag = false;
	publishSrcTrgCorrs_flag = false;
	executeLUM_flag = false;

	if (prop_store_first_cloud)
		add_cloud_flag = true;
	else
		add_cloud_flag = false;

	// Initialize variables.
	lum_xyzsift = pcl::registration::LUM<PointXYZSIFT>::Ptr (new pcl::registration::LUM<PointXYZSIFT>);

	return true;
}

bool CloudStorageLUM::onFinish() {
	return true;
}

bool CloudStorageLUM::onStop() {
	return true;
}

bool CloudStorageLUM::onStart() {
	return true;
}


void CloudStorageLUM::onAddCloudButtonPressed(){
	CLOG(LTRACE) << "onAddCloudButtonPressed";
	add_cloud_flag = true;
}

void CloudStorageLUM::onAddCloudTriggered(){
	CLOG(LDEBUG) << "onAddCloudTriggered";
	in_add_cloud_trigger.read();
	add_cloud_flag = true;
}


void CloudStorageLUM::onRemoveLastCloudButtonPressed(){
	CLOG(LTRACE) << "onRemoveLastCloudButtonPressed";
	remove_last_cloud_flag = true;
}


void CloudStorageLUM::onClearStorageButtonPressed(){
	CLOG(LTRACE) << "onClearStorageButtonPressed";
	clear_storage_flag = true;
}


void CloudStorageLUM::onPublishPreviousButtonPressed(){
	CLOG(LDEBUG) << "onPublishPreviousButtonPressed";
	publishPreviousCloud_flag = true;
}

void CloudStorageLUM::onReturnPreviousCloudTriggered(){
	CLOG(LDEBUG) << "onReturnPreviousCloudTriggered";
	in_publish_previous_cloud_trigger.read();
	publishPreviousCloud_flag = true;
}



void CloudStorageLUM::update_storage(){
	CLOG(LTRACE) << "update_storage";

	// Overwrite last cloud.
	if (prop_overwrite_last_cloud) {
		// Check if something can be added - if so, remove last cloud.
		if(!in_transformation.empty() && !in_cloud_xyzrgb.empty() && !in_cloud_xyzsift.empty()){
			remove_last_cloud_flag = true;
			add_cloud_flag = true;
		}//: if
		// otherwise - do nothing.
	}//: if

	// Remove last clouds and transformation from storage.
	if (remove_last_cloud_flag)
		remove_last_cloud_to_storage();
	
	// Add received clouds and transformation to storage.
	if (add_cloud_flag)
		add_cloud_to_storage();

	// Clear storage.
	if (clear_storage_flag)
		clear_storage();

	// Check size limit.
	if (prop_clouds_limit > 0){
		CLOG(LINFO) << "Limiting the size of stored clouds";
		while (transformations.size() > prop_clouds_limit) {
			// Remove first elements from vectors.
			transformations.erase(transformations.begin());
			if(!clouds_xyzrgb.empty())
				clouds_xyzrgb.erase(clouds_xyzrgb.begin());
			if(!clouds_xyzsift.empty())
				clouds_xyzsift.erase(clouds_xyzsift.begin());
		}//: if
	}//: if

	// Generate LUM graph SLAM.
	if(generateLumGraphSlam_flag)
		generateLumGraphSlam();

	// Publish LUM graph SLAM.
	if(publishLumGraphSlam_flag)
		publishLumGraphSlam();

	// Publish source and target clouds along with correspondences.
	if(publishSrcTrgCorrs_flag)
		publishSrcTrgCorrespondences();

	// Run LUM!
	if (executeLUM_flag)
		executeLUM();

	// Publish cloud merged from currently possesed ones.
	publish_merged_clouds();

	// Return previous (single or merged) cloud.
	if (publishPreviousCloud_flag)
		publishPreviousCloud();

}


void CloudStorageLUM::add_cloud_to_storage(){ 
	CLOG(LTRACE) << "add_cloud_to_storage";

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;

	try{ 
		// Check homogenous matrix and add to storage only if any of the clouds is present!
		if(in_transformation.empty() || in_cloud_xyzrgb.empty() || in_cloud_xyzsift.empty()){
			throw exception();
		}

		// Reset flag.
		add_cloud_flag = false;

		CLOG(LINFO) << "Adding transformation to storage";
		hm = in_transformation.read();
		transformations.push_back(hm);

		// Try to add XYZRGB.
		if(!in_cloud_xyzrgb.empty()){
			CLOG(LINFO) << "Adding XYZRGB cloud to storage";
			cloud_xyzrgb = in_cloud_xyzrgb.read();
			clouds_xyzrgb.push_back(cloud_xyzrgb);
		}//: if

		// Try to add XYZSIFT.
		if(!in_cloud_xyzsift.empty()){
			CLOG(LINFO) << "Adding XYZSIFT cloud to storage";
			cloud_xyzsift = in_cloud_xyzsift.read();
			clouds_xyzsift.push_back(cloud_xyzsift);
		}//: if

		CLOG(LINFO) << "ADD: transformations.size(): "<< transformations.size() <<  " clouds_xyzrgb.size(): "<< clouds_xyzrgb.size() << " clouds_xyzsift.size(): "<< clouds_xyzsift.size();
		CLOG(LNOTICE) << "Cloud added to storage - size: "<< transformations.size();
	} catch (...) {
		CLOG(LERROR) << "Cannot add clouds to storage - cloud transformation is required";
	}//: catch
}


void  CloudStorageLUM::remove_last_cloud_to_storage(){
	CLOG(LTRACE) << "remove_last_cloud_to_storage";
	// Reset flag.
	remove_last_cloud_flag = false;

	if (!transformations.empty())
		transformations.pop_back();
	if(!clouds_xyzrgb.empty())
		clouds_xyzrgb.pop_back();
	if(!clouds_xyzsift.empty())
		clouds_xyzsift.pop_back();
	CLOG(LINFO) << "REM: transformations.size(): "<< transformations.size() <<  " clouds_xyzrgb.size(): "<< clouds_xyzrgb.size() << " clouds_xyzsift.size(): "<< clouds_xyzsift.size();
	CLOG(LNOTICE) << "Previous cloud removed from storage - size: "<< transformations.size();
}


void CloudStorageLUM::clear_storage(){ 
	CLOG(LTRACE) << "clear_storage";
	// Clear clouds.
	transformations.clear();
	clouds_xyzrgb.clear();
	clouds_xyzsift.clear();
	// Reset flag.
	clear_storage_flag = false;
	CLOG(LNOTICE) << "Storage cleared";
}


void CloudStorageLUM::publish_merged_clouds(){
	CLOG(LTRACE) << "publish_merged_clouds";

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointXYZSIFT>::Ptr merged_cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);

	// Merge XYZRGB cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyzrgb.size()) {
		CLOG(LINFO) << "Sizes of transformation and clouds_xyzrgb vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = (clouds_xyzrgb[i]);
			Types::HomogMatrix tmp_hm = transformations[i];
			// Transform it.
			pcl::transformPointCloud(*tmp, *trans_tmp, tmp_hm);
			// Add to merged cloud.
			*merged_cloud_xyzrgb += *trans_tmp;
			CLOG(LDEBUG) << "cloud "<< i << " size="<<clouds_xyzrgb[i]->size() << " hm:\n" << tmp_hm;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyzrgb->size(): "<< merged_cloud_xyzrgb->size();
		out_cloud_xyzrgb.write(merged_cloud_xyzrgb);
	}//: else

	// Merge XYZSIFT cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyzsift.size()) {
		CLOG(LINFO) << "Sizes of transformation and clouds_xyzsift vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<PointXYZSIFT>::Ptr trans_tmp (new pcl::PointCloud<PointXYZSIFT>);
			pcl::PointCloud<PointXYZSIFT>::Ptr tmp = (clouds_xyzsift[i]);
			Types::HomogMatrix tmp_hm = transformations[i];
			// Transform it.
			pcl::transformPointCloud(*tmp, *trans_tmp, tmp_hm);
			// Add to merged cloud.
			*merged_cloud_xyzsift += *trans_tmp;
			CLOG(LDEBUG) << "cloud "<< i << " size="<<clouds_xyzsift[i]->size() << " hm:\n" << tmp_hm;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyzsift->size(): "<< merged_cloud_xyzsift->size();
		out_cloud_xyzsift.write(merged_cloud_xyzsift);
	}//: else


}

void CloudStorageLUM::publishPreviousCloud(){
	CLOG(LTRACE) << "publishPreviousCloud";
	publishPreviousCloud_flag = false;

	CLOG(LDEBUG) << "transformations.size(): "<< transformations.size() << " clouds_xyzrgb.size(): "<< clouds_xyzrgb.size();

	if (transformations.size() != clouds_xyzrgb.size()) {
		CLOG(LINFO) << "Sizes of transformation and clouds_xyzrgb vectors differ!";
		return;
	}//: if

	if (transformations.size() == 0) {
		CLOG(LINFO) << "There are no clouds to return!";
		return;
	}//: if

	if (prop_return_previous_merged_cloud) {
		// Return merged cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_previous (new pcl::PointCloud<pcl::PointXYZRGB>);
		// Merge all clouds EXCEPT the last (i.e. pairwise registered) one!
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = (clouds_xyzrgb[i]);
			// Transform it.
			pcl::transformPointCloud(*tmp, *trans_tmp, transformations[i]);
			// Add to merged cloud.
			*merged_previous += *trans_tmp;
		}
		// Return merged previous cloud.
		CLOG(LINFO) << "merged_previous->size(): "<< merged_previous->size();
		out_previous_cloud_xyzrgb.write(merged_previous);
	} else {
		// Return previous cloud.
		int i = transformations.size()-1;
		// Get previous (n-1) cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = (clouds_xyzrgb[i]);
		// Transform it.
		pcl::transformPointCloud(*tmp, *trans_tmp, transformations[i]);

		// Return previous cloud.
		out_previous_cloud_xyzrgb.write(trans_tmp);
	}//: else
}


void CloudStorageLUM::onPublishLUMGraphSLAMButtonPressed(){
	CLOG(LTRACE) << "onPublishLUMGraphSLAMButtonPressed()";
	publishLumGraphSlam_flag = true;
}


void CloudStorageLUM::publishLumGraphSlam() {
	CLOG(LTRACE) << "publishLumGraphSlam()";
	publishLumGraphSlam_flag = false;
	out_lum_xyzsift.write(lum_xyzsift);
}



void CloudStorageLUM::onGenerateLUMGraphSLAMButtonPressed(){
	CLOG(LTRACE) << "onGenerateLUMGraphSLAMButtonPressed()";
	generateLumGraphSlam_flag = true;
}

void CloudStorageLUM::generateLumGraphSlam() {
	CLOG(LTRACE) << "generateLumGraphSlam()";
	generateLumGraphSlam_flag = false;

	// Generate LUM structure - from scract, thus create new object.
	lum_xyzsift = pcl::registration::LUM<PointXYZSIFT>::Ptr (new pcl::registration::LUM<PointXYZSIFT>);


	// Add first cloud without transformation.
	lum_xyzsift->addPointCloud(clouds_xyzsift[0]);

	// Add graph vertices - clouds and transformations.
	for (int i = 1 ; i < transformations.size(); i++) {
		CLOG(LDEBUG) << "Adding cloud and transformation to lum, i="<<i;
		// Transform HM to acceptable format. :]
		float x,y,z,roll,pitch,yaw;
		Eigen::Affine3f aff3f = transformations[i];
		pcl::getTranslationAndEulerAngles(aff3f, x,y,z,roll,pitch,yaw);
		Eigen::Vector6f v6f;
		v6f << x,y,z,roll,pitch,yaw;
		// Add cloud and transformation.
		lum_xyzsift->addPointCloud(clouds_xyzsift[i], v6f);
	}//: for

	CLOG(LNOTICE) << "Generated graph with vertices: "<<lum_xyzsift->getNumVertices();
}


void CloudStorageLUM::onUpdateTransfromationsBasingOnLumGraphSlam() {
	CLOG(LTRACE) << "onUpdateTransfromationsBasingOnLumGraphSlam()";
	// Read lum.
	lum_xyzsift = in_lum_xyzsift.read();
	CLOG(LNOTICE) << "LUM Graph SLAM object updated";
}


void CloudStorageLUM::onPublishSrcTrgCorrespondencesButtonPressed() {
	CLOG(LTRACE) << "onPublishSrcTrgCorrespondencesButtonPressed()";
	publishSrcTrgCorrs_flag = true;
}


void CloudStorageLUM::publishSrcTrgCorrespondences() {
	CLOG(LTRACE) << "Number of vertices in graph = "<< lum_xyzsift->getNumVertices() << " prop_src_cloud_index = "<< prop_src_cloud_index << " prop_trg_cloud_index = " << prop_trg_cloud_index;
	publishSrcTrgCorrs_flag = false;
	// Check if there are any vertices added.
	if (lum_xyzsift->getNumVertices() == 0) {
		CLOG(LWARNING) << "The LUM Graph SLAM object is empty - please add clouds and generate it first!";
		return;
	}//: if

	// Check src and trg indices.
	if ((prop_src_cloud_index >= lum_xyzsift->getNumVertices()) || (prop_trg_cloud_index >= lum_xyzsift->getNumVertices())) {
		CLOG(LWARNING) << "The source and target clouds indices are out of range - size of LUM graph is: " << lum_xyzsift->getNumVertices();
		return;
	}//: if

	if (prop_trg_cloud_index <= prop_src_cloud_index) {
		CLOG(LWARNING) << "The source index must be smaller than the target index";
		return;
	}//: if


	out_src_cloud_xyzrgb.write(clouds_xyzrgb[prop_src_cloud_index]);
	out_src_cloud_xyzsift.write(clouds_xyzsift[prop_src_cloud_index]);

	out_trg_cloud_xyzrgb.write(clouds_xyzrgb[prop_trg_cloud_index]);
	out_trg_cloud_xyzsift.write(clouds_xyzsift[prop_trg_cloud_index]);
	//lum_xyzsift->getPointCloud(prop_trg_cloud_index)

	out_src_trg_correspondences.write(lum_xyzsift->getCorrespondences(prop_src_cloud_index, prop_trg_cloud_index));

	CLOG(LNOTICE) << "Published source ("<< clouds_xyzrgb[prop_src_cloud_index]->size() <<";"<< clouds_xyzsift[prop_src_cloud_index]->size() <<") and target ("<<
		clouds_xyzrgb[prop_trg_cloud_index]->size() <<";"<<clouds_xyzsift[prop_trg_cloud_index]->size() <<") clouds along with correspondences ("<< 
		lum_xyzsift->getCorrespondences(prop_src_cloud_index, prop_trg_cloud_index)->size() <<") ";
}

void CloudStorageLUM::onExecuteLUMButtonPressed() {
	CLOG(LTRACE) << "onExecuteLUMButtonPressed()";
	executeLUM_flag = true;
}

void CloudStorageLUM::executeLUM() {
	CLOG(LTRACE) << "executeLUM()";
	executeLUM_flag = false;

	// Set lum properties.
	//lum_xyzsift->setMaxIterations(maxIterations);
	
	// Execute LUM.
	CLOG(LNOTICE) << "Executing LUM Graph SLAM - please wait...";
	lum_xyzsift->compute();

	// Update transformations.
	for (int i = 1 ; i < transformations.size(); i++) {
		// CLOG(LDEBUG) << "Updating transformation on the basis of retrieved lum, i="<<i;
		Eigen::Affine3f aff3f = lum_xyzsift->getTransformation(i);
		CLOG(LDEBUG) << "transformations["<<i<<"] before=\n"<<transformations[i];
		transformations[i] = aff3f;
		CLOG(LDEBUG) << "transformations["<<i<<"] after=\n"<<transformations[i];
	}//: for
	CLOG(LNOTICE) << "Stored dataset refined with LUM Graph SLAM!";

}

} //: namespace CloudStorageLUM
} //: namespace Processors
