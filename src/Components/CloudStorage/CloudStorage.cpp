/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "CloudStorage.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace CloudStorage {

CloudStorage::CloudStorage(const std::string & name) :
	Base::Component(name),
	prop_store_first_cloud("StoreFirstCloud",true),
	prop_overwrite_last_cloud("OverwriteLastCloud",false),
	prop_clouds_limit("CloudsLimit",0)
{
	// Register properties.
	registerProperty(prop_store_first_cloud);
	registerProperty(prop_overwrite_last_cloud);
	registerProperty(prop_clouds_limit);
	prop_clouds_limit.addConstraint("0");
	prop_clouds_limit.addConstraint("999");
}

CloudStorage::~CloudStorage() {
}

void CloudStorage::prepareInterface() {
	// Register input data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_transformation", &in_transformation);
	registerStream("in_add_cloud_trigger", &in_add_cloud_trigger);

	// Register output data streams.
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register button-triggered handlers.
	registerHandler("Add cloud", boost::bind(&CloudStorage::onAddCloudButtonPressed, this));
	registerHandler("Remove last cloud", boost::bind(&CloudStorage::onRemoveLastCloudButtonPressed, this));
	registerHandler("Clear storage", boost::bind(&CloudStorage::onClearStorageButtonPressed, this));

	// Register externally-triggered handler.
	registerHandler("onAddCloudTriggered", boost::bind(&CloudStorage::onAddCloudTriggered, this));
	addDependency("onAddCloudTriggered", &in_add_cloud_trigger);

	// Registed "main" storage management method.
	registerHandler("update_storage", boost::bind(&CloudStorage::update_storage, this));
	addDependency("update_storage", NULL);
}

bool CloudStorage::onInit() {
	// Init flags.
	remove_last_cloud_flag = false;
	clear_storage_flag = false;

	if (prop_store_first_cloud)
		add_cloud_flag = true;
	else
		add_cloud_flag = false;

	return true;
}

bool CloudStorage::onFinish() {
	return true;
}

bool CloudStorage::onStop() {
	return true;
}

bool CloudStorage::onStart() {
	return true;
}


void CloudStorage::onAddCloudButtonPressed(){
	CLOG(LTRACE) << "CloudStorage::onAddCloudButtonPressed";
	add_cloud_flag = true;
}

void CloudStorage::onAddCloudTriggered(){
	CLOG(LDEBUG) << "CloudStorage::onAddCloudTriggered";
	in_add_cloud_trigger.read();
	add_cloud_flag = true;
}


void CloudStorage::onRemoveLastCloudButtonPressed(){
	CLOG(LTRACE) << "CloudStorage::onRemoveLastCloudButtonPressed";
	remove_last_cloud_flag = true;
}


void CloudStorage::onClearStorageButtonPressed(){
	CLOG(LTRACE) << "CloudStorage::onClearStorageButtonPressed";
	clear_storage_flag = true;
}


void CloudStorage::update_storage(){
	CLOG(LTRACE) << "CloudStorage::update_storage";

	// Overwrite last cloud.
	if (prop_overwrite_last_cloud) {
		// Check if something can be added - if so, remove last cloud.
		if(!in_transformation.empty() && (!in_cloud_xyz.empty() || !in_cloud_xyzrgb.empty())){
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
			if(!clouds_xyz.empty())
				clouds_xyz.erase(clouds_xyz.begin());
			if(!clouds_xyzrgb.empty())
				clouds_xyzrgb.erase(clouds_xyzrgb.begin());
		}//: if
	}//: if

	// Publish cloud merged from currently possesed ones.
	publish_merged_clouds();
}


void CloudStorage::add_cloud_to_storage(){ 
	CLOG(LTRACE) << "CloudStorage::add_cloud_to_storage";

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;

	try{ 
		// Check homogenous matrix and add to storage only if any of the clouds is present!
		if(in_transformation.empty() || (in_cloud_xyz.empty() && in_cloud_xyzrgb.empty())){
			throw exception();
		}

		// Reset flag.
		add_cloud_flag = false;

		CLOG(LINFO) << "Adding transformation to storage";
		hm = in_transformation.read();
		transformations.push_back(hm);

		// Try to add XYZ.
		if(!in_cloud_xyz.empty()){
			CLOG(LINFO) << "Adding XYZ cloud to storage";
			cloud_xyz = in_cloud_xyz.read();
			clouds_xyz.push_back(cloud_xyz);
		}//: if

		// Try to add XYZRGB.
		if(!in_cloud_xyzrgb.empty()){
			CLOG(LINFO) << "Adding XYZRGB cloud to storage";
			cloud_xyzrgb = in_cloud_xyzrgb.read();
			clouds_xyzrgb.push_back(cloud_xyzrgb);
		}//: if

	CLOG(LNOTICE) << "ADD: transformations.size(): "<< transformations.size() << " clouds_xyz.size(): "<< clouds_xyz.size() << " clouds_xyzrgb.size(): "<< clouds_xyzrgb.size();
	} catch (...) {
		CLOG(LERROR) << "Cannot add clouds to storage - cloud transformation is required";
	}//: catch
}


void  CloudStorage::remove_last_cloud_to_storage(){
	CLOG(LTRACE) << "CloudStorage::remove_last_cloud_to_storage";
	// Reset flag.
	remove_last_cloud_flag = false;

	if (!transformations.empty())
		transformations.pop_back();
	if(!clouds_xyz.empty())
		clouds_xyz.pop_back();
	if(!clouds_xyzrgb.empty())
		clouds_xyzrgb.pop_back();
	CLOG(LNOTICE) << "REM: transformations.size(): "<< transformations.size() << " clouds_xyz.size(): "<< clouds_xyz.size() << " clouds_xyzrgb.size(): "<< clouds_xyzrgb.size();
}


void CloudStorage::clear_storage(){ 
	CLOG(LTRACE) << "CloudStorage::clear_storage";
	transformations.clear();
	clouds_xyz.clear();
	clouds_xyzrgb.clear();
	// Reset flag.
	clear_storage_flag = false;
}


void  CloudStorage::publish_merged_clouds(){
	CLOG(LTRACE) << "CloudStorage::publish_merged_clouds";

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Check size of vector of transformations.
/*	if (transformations.size() < 1) {
		CLOG(LINFO) << "Vector of transformations is empty";
		return;
	}//: if*/


	// Merge XYZ cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyz.size()) {
		CLOG(LINFO) << "Sizes of transformation and clouds_xyz vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZ> tmp = *(clouds_xyz[i]);
			// Transform it.
			pcl::transformPointCloud(tmp, tmp, transformations[i].getElements());
			// Add to merged cloud.
			*merged_cloud_xyz += tmp;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyz->size(): "<< merged_cloud_xyz->size();
		out_cloud_xyz.write(merged_cloud_xyz);
	}//: else


	// Merge XYZRGB cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyzrgb.size()) {
		CLOG(LINFO) << "Sizes of transformation and clouds_xyzrgb vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZRGB> tmp = *(clouds_xyzrgb[i]);
			// Transform it.
			pcl::transformPointCloud(tmp, tmp, transformations[i].getElements());
			// Add to merged cloud.
			*merged_cloud_xyzrgb += tmp;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyzrgb->size(): "<< merged_cloud_xyzrgb->size();
		out_cloud_xyzrgb.write(merged_cloud_xyzrgb);
	}//: else


}



} //: namespace CloudStorage
} //: namespace Processors
