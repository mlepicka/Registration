/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "PairwiseRegistration.hpp"
#include "Common/Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/bind.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

// ICP with normals
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>

// ICP with colour
#include "Types/CorrespondenceEstimationColor.hpp"

//ICP with SIFT and KAZE features
#include "Types/CorrespondanceEstimationSIFT.hpp"
#include "Types/CorrespondanceEstimationKAZE.hpp"

#include <pcl/registration/correspondence_estimation.h>

namespace Processors {
namespace PairwiseRegistration {

const double NORMALS_RADIUS = 0.04;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

PairwiseRegistration::PairwiseRegistration(const std::string & name) :
		Base::Component(name), prop_ICP("Mode.ICP", true), prop_ICP_MaxCorrespondenceDistance(
				"ICP.MaxCorrespondenceDistance", 0.05), prop_ICP_MaximumIterations(
				"ICP.MaximumIterations", 50), prop_ICP_TransformationEpsilon(
				"ICP.TransformationEpsilon", 1e-8), prop_ICP_EuclideanFitnessEpsilon(
				"ICP.EuclideanFitnessEpsilon", 1), prop_ICP_colour(
				"ICP.UseColour", true), prop_ICP_normals("ICP.UseNormals",
				true), prop_ICP_SIFT("ICP.UseSIFT", true), prop_ICP_KAZE(
				"ICP.UseKAZE", true), prop_calc_path("Calculations.path",
				std::string("")) {
	// Register ICP properties.
	registerProperty(prop_ICP);
	registerProperty(prop_ICP_MaxCorrespondenceDistance);
	registerProperty(prop_ICP_MaximumIterations);
	prop_ICP_MaximumIterations.addConstraint("1");
	prop_ICP_MaximumIterations.addConstraint("999");
	registerProperty(prop_ICP_TransformationEpsilon);
	registerProperty(prop_ICP_EuclideanFitnessEpsilon);
	registerProperty(prop_ICP_colour);
	registerProperty(prop_ICP_normals);
	registerProperty(prop_ICP_SIFT);
	registerProperty(prop_ICP_KAZE);
	registerProperty(prop_calc_path);
}

PairwiseRegistration::~PairwiseRegistration() {
}

void PairwiseRegistration::prepareInterface() {
	// Register data streams.
	registerStream("in_trg_cloud_xyzrgb", &in_trg_cloud_xyzrgb);
	registerStream("in_src_cloud_xyzrgb", &in_src_cloud_xyzrgb);
	registerStream("in_save_src_cloud_trigger", &in_save_src_cloud_trigger);
	registerStream("out_transformation_xyz", &out_transformation_xyz);
	registerStream("out_transformation_xyzrgb", &out_transformation_xyzrgb);
	registerStream("in_correspondences", &in_src_trg_correspondences);
	registerStream("in_src_cloud_xyzsift", &in_src_cloud_xyzsift);
	registerStream("in_trg_cloud_xyzsift", &in_trg_cloud_xyzsift);
	registerStream("in_src_cloud_xyzkaze", &in_src_cloud_xyzkaze);
	registerStream("in_trg_cloud_xyzkaze", &in_trg_cloud_xyzkaze);
	registerStream("in_trg_cloud_xyzrgb_original",
			&in_trg_cloud_xyzrgb_original);
	registerStream("in_src_cloud_xyzrgb_original",
			&in_src_cloud_xyzrgb_original);

	registerHandler("pairwise_registration_xyzsift",
			boost::bind(&PairwiseRegistration::pairwise_registration_xyzsift,
					this));
	addDependency("pairwise_registration_xyzsift", &in_trg_cloud_xyzsift);
	addDependency("pairwise_registration_xyzsift", &in_src_cloud_xyzsift);

	registerHandler("pairwise_registration_xyzkaze",
			boost::bind(&PairwiseRegistration::pairwise_registration_xyzkaze,
					this));
	addDependency("pairwise_registration_xyzkaze", &in_trg_cloud_xyzkaze);
	addDependency("pairwise_registration_xyzkaze", &in_src_cloud_xyzkaze);

	// Register handlers
	registerHandler("pairwise_registration_xyzrgb",
			boost::bind(&PairwiseRegistration::pairwise_registration_xyzrgb,
					this));
	addDependency("pairwise_registration_xyzrgb", &in_trg_cloud_xyzrgb);

}

bool PairwiseRegistration::onInit() {
	counter = 0;
	// Init prev cloud.
	src_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	src_cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr(
			new pcl::PointCloud<PointXYZSIFT>);
	src_cloud_xyzkaze = pcl::PointCloud<PointXYZKAZE>::Ptr(
			new pcl::PointCloud<PointXYZKAZE>);
	return true;
}

bool PairwiseRegistration::onFinish() {
	return true;
}

bool PairwiseRegistration::onStop() {
	return true;
}

bool PairwiseRegistration::onStart() {
	return true;
}

void PairwiseRegistration::save_calculations(const double nr_iterations,
		const double size_of_cloud, double time, double fitness_score,
		double correspondences) {
	if (!in_save_src_cloud_trigger.empty() && !string(prop_calc_path).empty()) {
		CLOG(LINFO)<< "Saving calculations in path: " << string(prop_calc_path) << endl;
		std::ofstream iterations, cloud_size, cloud_size_original, proceed_time, fitness_score_value, nr_correspondences;

		iterations.open((string(prop_calc_path) + string("ilosc_iteracji.txt")).c_str(),
				ios::out | ios::app);

		cloud_size.open(
				(string(prop_calc_path) + string("ilosc_punktow_w_chmurze.txt")).c_str(),
				ios::out | ios::app);

		if(!in_src_cloud_xyzrgb_original.empty()) {
			cloud_size_original.open(
					(string(prop_calc_path) + string("ilosc_punktow_w_chmurze_oryginalnej.txt")).c_str(),
					ios::out | ios::app);
		}

		proceed_time.open((string(prop_calc_path) + string("czas_icp.txt")).c_str(),
				ios::out | ios::app);

		fitness_score_value.open((string(prop_calc_path) + string("wartosc_funkcji_bledu.txt")).c_str(),
				ios::out | ios::app);

		nr_correspondences.open((string(prop_calc_path) + string("znalezione_dopasowania.txt")).c_str(),
				ios::out | ios::app);

		iterations << nr_iterations << endl;
		cloud_size << size_of_cloud << endl;

		if(!in_src_cloud_xyzrgb_original.empty()) {
			src_cloud_xyzrgb = in_src_cloud_xyzrgb_original.read();
			cloud_size_original << src_cloud_xyzrgb->size() << endl;
		}

		proceed_time << time << endl;
		fitness_score_value << fitness_score << endl;
		nr_correspondences << correspondences << endl;

		iterations.close();
		cloud_size.close();

		if(!in_src_cloud_xyzrgb_original.empty()){
			cloud_size_original.close();
		}

		proceed_time.close();
		fitness_score_value.close();
		nr_correspondences.close();
	}
}
void PairwiseRegistration::save_original_distances(Types::HomogMatrix& result) {
	if (!in_save_src_cloud_trigger.empty() && string(prop_calc_path).empty()) {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_tmp(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud;

		if (!in_trg_cloud_xyzrgb_original.empty()
				&& !in_src_cloud_xyzrgb_original.empty()) {
			tmp = in_trg_cloud_xyzrgb_original.read();
			src_cloud = in_src_cloud_xyzrgb_original.read();
		} else if (!in_trg_cloud_xyzrgb.empty()
				&& !in_src_cloud_xyzrgb.empty()) {
			tmp = in_trg_cloud_xyzrgb.read();
			src_cloud = in_src_cloud_xyzrgb.read();
		} else {
			return;
		}

		// Transform it.
		pcl::transformPointCloud(*tmp, *trans_tmp, result);
		Eigen::Matrix<float, 3, 1> src_points, tgt_points;
		fstream dists_in_cloud;
		stringstream name;
		name << string(prop_calc_path) << "punkty/odeglosci_widok_nr_"
				<< counter << ".txt";
		dists_in_cloud.open(name.str().c_str(), ios::out | ios::app);
		for (int i = 0; i < src_cloud.get()->points.size(); i++) {
			src_points << src_cloud.get()->points[i].x, src_cloud.get()->points[i].y, src_cloud.get()->points[i].z;
			tgt_points << trans_tmp.get()->points[i].x, trans_tmp.get()->points[i].y, trans_tmp.get()->points[i].z;
			float d = (src_points - tgt_points).norm();
			dists_in_cloud << d << endl;
		}
		in_save_src_cloud_trigger.read();
		counter++;
	}
}
template<typename T> void PairwiseRegistration::set_parameters(
		pcl::IterativeClosestPoint<T, T>& icp) {
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(prop_ICP_MaxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(prop_ICP_MaximumIterations);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(prop_ICP_TransformationEpsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(prop_ICP_EuclideanFitnessEpsilon);
}

Types::HomogMatrix PairwiseRegistration::pairwise_icp_based_registration_xyzsift(
		pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_xyzsift_,
		pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsift_) {
	CLOG(LTRACE)<< "pairwise_icp_based_registration_xyzsift_";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;

	if(prop_ICP_SIFT) {
		Common::Timer timer;
		pcl::IterativeClosestPoint<PointXYZSIFT, PointXYZSIFT> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		set_parameters<PointXYZSIFT>(icp);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationSIFT<PointXYZSIFT, PointXYZSIFT, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationSIFT<PointXYZSIFT, PointXYZSIFT, float>);
		icp.setCorrespondenceEstimation(ceptr);

		icp.setInputSource (src_cloud_xyzsift_);
		icp.setInputTarget (trg_cloud_xyzsift_);

		pcl::PointCloud<PointXYZSIFT>::Ptr aligned_trg_cloud_xyzsift_ (new pcl::PointCloud<PointXYZSIFT>());

		// Align clouds.
		timer.restart();
		icp.align(*aligned_trg_cloud_xyzsift_);
		double time = timer.elapsed();

		Types::HomogMatrix icp_trans = icp.getFinalTransformation();

		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
		<< " iterations: " << icp.nr_iterations_
		<< " correspondences: " << icp.correspondences_->size();

		save_calculations(icp.nr_iterations_, src_cloud_xyzsift_->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	}
	return result;

}

Types::HomogMatrix PairwiseRegistration::pairwise_icp_based_registration_xyzkaze(
		pcl::PointCloud<PointXYZKAZE>::Ptr src_cloud_xyzkaze_,
		pcl::PointCloud<PointXYZKAZE>::Ptr trg_cloud_xyzkaze_) {
	CLOG(LTRACE)<< "pairwise_icp_based_registration_xyzkaze_";
// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;

	if(prop_ICP_KAZE) {
		Common::Timer timer;
		pcl::IterativeClosestPoint<PointXYZKAZE, PointXYZKAZE> icp;

		set_parameters<PointXYZKAZE>(icp);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationKAZE<PointXYZKAZE, PointXYZKAZE, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationKAZE<PointXYZKAZE, PointXYZKAZE, float>);
		icp.setCorrespondenceEstimation(ceptr);

		icp.setInputSource (src_cloud_xyzkaze_);
		icp.setInputTarget (trg_cloud_xyzkaze_);

		pcl::PointCloud<PointXYZKAZE>::Ptr aligned_trg_cloud_xyzkaze_ (new pcl::PointCloud<PointXYZKAZE>());

		// Align clouds.
		timer.restart();
		icp.align(*aligned_trg_cloud_xyzkaze_);
		double time = timer.elapsed();

		Types::HomogMatrix icp_trans = icp.getFinalTransformation();

		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
		<< " iterations: " << icp.nr_iterations_
		<< " correspondences: " << icp.correspondences_->size();

		save_calculations(icp.nr_iterations_, src_cloud_xyzkaze_->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();
	}
	return result;

}

Types::HomogMatrix PairwiseRegistration::pairwise_icp_based_registration_xyzrgb(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_xyzrgb_,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb_) {
	CLOG(LTRACE)<< "pairwise_icp_based_registration_xyzrgb";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;

	// Perform pairwise registration.
	if (prop_ICP_colour && prop_ICP_normals) {

		CLOG(LINFO) << "Using ICP with colour and normals for pairwise registration";

		// Compute surface normals.
		pcl::PointCloud<pcl::Normal>::Ptr tmp_trg_cloud_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		norm_est.setSearchMethod (tree);
		norm_est.setKSearch (30);
		//norm_est.setRadiusSearch(radius_search); // 0.05

		// Compute normals for previous cloud.
		norm_est.setInputCloud (src_cloud_xyzrgb_);
		norm_est.compute (*tmp_trg_cloud_normal);
		// Concatenate clouds containing XYZRGB points and normals.
		pcl::concatenateFields(*src_cloud_xyzrgb_, *tmp_trg_cloud_normal, *src_cloud_xyzrgbnormal);

		// Compute normals for transformed cloud.
		norm_est.setInputCloud (trg_cloud_xyzrgb_);
		norm_est.compute (*tmp_trg_cloud_normal);
		// Concatenate clouds containing XYZRGB points and normals.
		pcl::concatenateFields(*trg_cloud_xyzrgb_, *tmp_trg_cloud_normal, *trg_cloud_xyzrgbnormal);

		// Use ICP with normals AND colour to get "better" transformation.
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

		set_parameters<pcl::PointXYZRGBNormal>(icp);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, float>);
		icp.setCorrespondenceEstimation(ceptr);

		icp.setInputSource (src_cloud_xyzrgbnormal);
		icp.setInputTarget (trg_cloud_xyzrgbnormal);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		// Align clouds.
		Common::Timer timer;
		timer.restart();
		icp.align(*aligned_trg_cloud_xyzrgbnormal);
		double time = timer.elapsed();

		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		save_calculations(icp.nr_iterations_, src_cloud_xyzrgbnormal->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();
		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	} else if (prop_ICP_normals) {
		CLOG(LINFO) << "Using ICP with normals for pairwise registration";

		// Compute surface normals.
		pcl::PointCloud<pcl::Normal>::Ptr tmp_trg_cloud_normal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		norm_est.setSearchMethod (tree);
		norm_est.setKSearch (30);
		//norm_est.setRadiusSearch(radius_search); // 0.05

		// Compute normals for previous cloud.
		norm_est.setInputCloud (src_cloud_xyzrgb_);
		norm_est.compute (*tmp_trg_cloud_normal);
		// Concatenate clouds containing XYZRGB points and normals.
		pcl::concatenateFields(*src_cloud_xyzrgb_, *tmp_trg_cloud_normal, *src_cloud_xyzrgbnormal);

		// Compute normals for transformed cloud.
		norm_est.setInputCloud (trg_cloud_xyzrgb_);
		norm_est.compute (*tmp_trg_cloud_normal);
		// Concatenate clouds containing XYZRGB points and normals.
		pcl::concatenateFields(*trg_cloud_xyzrgb_, *tmp_trg_cloud_normal, *trg_cloud_xyzrgbnormal);

		// Use ICP with normals to get "better" transformation.
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

		set_parameters<pcl::PointXYZRGBNormal>(icp);

		icp.setInputSource (src_cloud_xyzrgbnormal);
		icp.setInputTarget (trg_cloud_xyzrgbnormal);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		// Align clouds.
		Common::Timer timer;
		timer.restart();

		icp.align(*aligned_trg_cloud_xyzrgbnormal);

		double time = timer.elapsed();
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		save_calculations(icp.nr_iterations_, src_cloud_xyzrgbnormal->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();

		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	} else if (prop_ICP_colour) {
		CLOG(LINFO) << "Using ICP with colour for pairwise registration " << in_save_src_cloud_trigger.empty();
		// Use ICP with colour to get "better" transformation.
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		set_parameters<pcl::PointXYZRGB>(icp);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>);
		icp.setCorrespondenceEstimation(ceptr);

		// Add source and target clours.
		icp.setInputSource(src_cloud_xyzrgb_);
		icp.setInputTarget(trg_cloud_xyzrgb_);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_trg_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());

		// Align clouds.
		Common::Timer timer;
		timer.restart();
		icp.align(*aligned_trg_cloud_rgbxyz);
		double time = timer.elapsed();
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		save_calculations(icp.nr_iterations_, src_cloud_xyzrgb_->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();
		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	} else if (!prop_ICP_SIFT && !prop_ICP_KAZE) {
		CLOG(LINFO) << "Using stantard ICP for pairwise registration";
		// Use ICP to get "better" transformation.
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		set_parameters<pcl::PointXYZRGB>(icp);

		// Add source and target clours.
		icp.setInputSource(src_cloud_xyzrgb_);
		icp.setInputTarget(trg_cloud_xyzrgb_);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_trg_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());

		// Align clouds.
		Common::Timer timer;
		timer.restart();
		icp.align(*aligned_trg_cloud_rgbxyz);
		double time = timer.elapsed();
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		save_calculations(icp.nr_iterations_, src_cloud_xyzrgb_->size(), time, icp.getFitnessScore(), icp.correspondences_->size());

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();
		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	}		//: else ICP

	// Return transformation.
	return result;
}

void PairwiseRegistration::pairwise_registration_xyzrgb() {
	if (!prop_ICP_SIFT && !prop_ICP_KAZE) {
		CLOG(LTRACE)<< "pairwise_registration_xyzrgb";
		// Temporary variable storing the resulting transformation.
		Types::HomogMatrix result;
		// Read current cloud from port.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb = in_trg_cloud_xyzrgb.read();

		// Check whether source cloud is received and is required to save.
		if ((!in_src_cloud_xyzrgb.empty()) && (!in_save_src_cloud_trigger.empty())) {
			CLOG(LINFO) << "Storing source cloud";
			src_cloud_xyzrgb = in_src_cloud_xyzrgb.read();
			//in_save_src_cloud_trigger.read();
			// pcl::copyPointCloud<pcl::PointXYZRGB> (*trg_cloud_xyzrgb, *src_cloud_xyzrgb);
		}		//: if

		// Debug.
		CLOG(LDEBUG) << "Target cloud size:" << trg_cloud_xyzrgb->size();
		if (!src_cloud_xyzrgb->empty ())
		CLOG(LDEBUG) << "Source cloud size:" << src_cloud_xyzrgb->size();

		/// Previous cloud empty - initialization or ICP-based pairwise registration should not be used.
		if (src_cloud_xyzrgb->empty () || !prop_ICP) {
			// Return identity matrix.
			CLOG(LINFO) << "ICP refinement not used";
			result.setIdentity();
		} else {
			// Run ICP.
			result = pairwise_icp_based_registration_xyzrgb(src_cloud_xyzrgb, trg_cloud_xyzrgb);

			save_original_distances(result);

		}		//: else - !src_cloud_xyzrgb->empty ()

		// Return the transformation.

		out_transformation_xyzrgb.write(result);
	}
	else {
		out_transformation_xyzrgb.write(this->result);
	}
}

void PairwiseRegistration::pairwise_registration_xyzsift() {
	CLOG(LTRACE)<< "pairwise_registration_xyzsift";

	if(prop_ICP_SIFT) {
		// Temporary variable storing the resulting transformation.
		Types::HomogMatrix result;
		// Read current cloud from port.
		pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsift = in_trg_cloud_xyzsift.read();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb = in_trg_cloud_xyzrgb.read();

		CLOG(LINFO) << "Reading src_cloud!";
		src_cloud_xyzsift = in_src_cloud_xyzsift.read();
		// Check whether source cloud is received and is required to save.
		if ((!in_src_cloud_xyzsift.empty()) && (!in_save_src_cloud_trigger.empty())) {
			CLOG(LINFO) << "Storing source cloud";
			src_cloud_xyzsift = in_src_cloud_xyzsift.read();
			in_save_src_cloud_trigger.read();
			// pcl::copyPointCloud<pcl::PointXYZRGB> (*trg_cloud_xyzrgb, *src_cloud_xyzrgb);
		}		//: if

		// Debug.
		CLOG(LDEBUG) << "Target cloud size:" << trg_cloud_xyzsift->size();
		if (!src_cloud_xyzsift->empty ())
		CLOG(LDEBUG) << "Source cloud size:" << src_cloud_xyzsift->size();

		/// Previous cloud empty - initialization or ICP-based pairwise registration should not be used.
		if (src_cloud_xyzsift->empty () || !prop_ICP) {
			// Return identity matrix.
			CLOG(LINFO) << "ICP refinement not used";
			result.setIdentity();
		} else {
			// Run ICP.
			result = pairwise_icp_based_registration_xyzsift(src_cloud_xyzsift, trg_cloud_xyzsift);

			save_original_distances(result);

		}		//: else - !src_cloud_xyzrgb->empty ()

		// Return the transformation.

		CLOG(LINFO) << "transformation: " << result;
		this->result = result;
		out_transformation_xyzrgb.write(result);
	}
}

void PairwiseRegistration::pairwise_registration_xyzkaze() {
	CLOG(LTRACE)<< "pairwise_registration_xyzkaze";
	if(prop_ICP_KAZE) {
		// Temporary variable storing the resulting transformation.
		Types::HomogMatrix result;
		// Read current cloud from port.
		pcl::PointCloud<PointXYZKAZE>::Ptr trg_cloud_xyzkaze = in_trg_cloud_xyzkaze.read();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb = in_trg_cloud_xyzrgb.read();

		CLOG(LINFO) << "Reading src_cloud!";
		src_cloud_xyzkaze = in_src_cloud_xyzkaze.read();

		// Check whether source cloud is received and is required to save.
		if ((!in_src_cloud_xyzkaze.empty()) && (!in_save_src_cloud_trigger.empty())) {
			CLOG(LINFO) << "Storing source cloud";
			src_cloud_xyzkaze = in_src_cloud_xyzkaze.read();
			in_save_src_cloud_trigger.read();
			// pcl::copyPointCloud<pcl::PointXYZRGB> (*trg_cloud_xyzrgb, *src_cloud_xyzrgb);
		}		//: if

		// Debug.
		CLOG(LDEBUG) << "Target cloud size:" << trg_cloud_xyzkaze->size();
		if (!src_cloud_xyzkaze->empty ())
			CLOG(LDEBUG) << "Source cloud size:" << src_cloud_xyzkaze->size();

		/// Previous cloud empty - initialization or ICP-based pairwise registration should not be used.
		if (src_cloud_xyzkaze->empty () || !prop_ICP) {
			// Return identity matrix.
			CLOG(LINFO) << "ICP refinement not used";
			result.setIdentity();
		} else {
			// Run ICP.
			result = pairwise_icp_based_registration_xyzkaze(src_cloud_xyzkaze, trg_cloud_xyzkaze);

			save_original_distances(result);

		}		//: else - !src_cloud_xyzrgb->empty ()

		// Return the transformation.
		CLOG(LINFO) << "transformation kaze: " << result;
		this->result = result;
		out_transformation_xyzrgb.write(result);
	}
}

} //: namespace PairwiseRegistration
} //: namespace Processors
