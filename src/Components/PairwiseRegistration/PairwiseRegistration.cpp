/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "PairwiseRegistration.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
// ICP
#include <pcl/registration/icp.h>

// ICP with normals
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>

// ICP with colour
#include "Types/CorrespondenceEstimationColor.hpp"
#include "Types/CorrespondanceEstimationSIFT.hpp"

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

/*pcl::PointCloud<pcl::Normal>::Ptr getNormals( pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud ) {

 pcl::PointCloud<pcl::Normal>::Ptr normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
 norm_est.setInputCloud( incloud );
 norm_est.setRadiusSearch( NORMALS_RADIUS );
 norm_est.compute( *normalsPtr );
 return normalsPtr;
 }
 */

PairwiseRegistration::PairwiseRegistration(const std::string & name) :
		Base::Component(name), prop_ICP("Mode.ICP", true), prop_ICP_MaxCorrespondenceDistance(
				"ICP.MaxCorrespondenceDistance", 0.05), prop_ICP_MaximumIterations(
				"ICP.MaximumIterations", 50), prop_ICP_TransformationEpsilon(
				"ICP.TransformationEpsilon", 1e-8), prop_ICP_EuclideanFitnessEpsilon(
				"ICP.EuclideanFitnessEpsilon", 1), prop_ICP_colour(
				"ICP.UseColour", true), prop_ICP_normals("ICP.UseNormals",
				true), prop_ICP_SIFT("ICP.UseSIFT", true) {
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

	registerHandler("pairwise_registration_xyzsift",
			boost::bind(&PairwiseRegistration::pairwise_registration_xyzsift,
					this));
	addDependency("pairwise_registration_xyzsift", &in_trg_cloud_xyzsift);
	addDependency("pairwise_registration_xyzsift", &in_src_trg_correspondences);
	//addDependency("pairwise_registration_xyzsift", &in_src_cloud_xyzsift);
	// Register handlers
	registerHandler("pairwise_registration_xyzrgb",
			boost::bind(&PairwiseRegistration::pairwise_registration_xyzrgb,
					this));
	addDependency("pairwise_registration_xyzrgb", &in_trg_cloud_xyzrgb);

}

bool PairwiseRegistration::onInit() {
	// Init prev cloud.
	src_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	src_cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr(
			new pcl::PointCloud<PointXYZSIFT>);
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

Types::HomogMatrix PairwiseRegistration::pairwise_icp_based_registration_xyzsift(
		pcl::PointCloud<PointXYZSIFT>::Ptr src_cloud_xyzsift_,
		pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsift_,
		pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT>::Ptr correspondences) {
	CLOG(LTRACE)<< "pairwise_icp_based_registration_xyzsift_";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;

	if(prop_ICP_SIFT) {
		//pcl::CorrespondencesPtr correspondences	= estimateCorrespondences(src_cloud_xyzsift_, trg_cloud_xyzsift_);
		pcl::IterativeClosestPoint<PointXYZSIFT, PointXYZSIFT> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (prop_ICP_MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationSIFT<PointXYZSIFT, PointXYZSIFT, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationSIFT<PointXYZSIFT, PointXYZSIFT, float>);
		icp.setCorrespondenceEstimation(ceptr);

		//SIFTFeatureRepresentation::Ptr point_representation(
		//						new SIFTFeatureRepresentation());
		//icp.setPointRepresentation(point_representation);

		icp.setInputSource (src_cloud_xyzsift_);
		icp.setInputTarget (trg_cloud_xyzsift_);

		pcl::PointCloud<PointXYZSIFT>::Ptr aligned_trg_cloud_xyzsift_ (new pcl::PointCloud<PointXYZSIFT>());

		// Align clouds.
		for(int i=0; i<10; i++){
			icp.align(*aligned_trg_cloud_xyzsift_);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
					<< " iterations: " << icp.nr_iterations_
					<< " correspondences: " << icp.correspondences_->size();
		}
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
		<< " iterations: " << icp.nr_iterations_ ;

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();
		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

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
//		pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (prop_ICP_MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, float>);
		icp.setCorrespondenceEstimation(ceptr);

		icp.setInputSource (src_cloud_xyzrgbnormal);
		icp.setInputTarget (trg_cloud_xyzrgbnormal);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		// Align clouds.
		icp.align(*aligned_trg_cloud_xyzrgbnormal);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

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
//		pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (2);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		// Set the point representation - x,y,z and curvature.
		//icp.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

		icp.setInputSource (src_cloud_xyzrgbnormal);
		icp.setInputTarget (trg_cloud_xyzrgbnormal);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_trg_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

		/*
		 // Run the same optimization in a loop and visualize the results
		 Types::HomogMatrix Ti = Types::HomogMatrix::Identity (), prev, targetToSource;
		 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		 reg_result = src_cloud_xyzrgbnormals;

		 for (int i = 0; i < prop_ICP_MaximumIterations; ++i)
		 {
		 CLOG(LINFO) << "ICP normals iteration:" << i;
		 // Estimate
		 icp.setInputSource(reg_result);
		 icp.align (*aligned_trg_cloud_rgbxyzn);
		 if(icp.converged_==false)
		 {
		 CLOG(LERROR) << "Not enough correspondences found!";
		 return;
		 }
		 //accumulate transformation between each Iteration
		 Ti = icp.getFinalTransformation () * Ti;

		 //if the difference between this transformation and the previous one
		 //is smaller than the threshold, refine the process by reducing
		 //the maximal correspondence distance
		 if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
		 icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);
		 prev = icp.getLastIncrementalTransformation ();
		 reg_result= Final;
		 }
		 Types::HomogMatrix icp_trans = Ti;*/

		// Align clouds.
		icp.align(*aligned_trg_cloud_xyzrgbnormal);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();

		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	} else if (prop_ICP_colour) {
		CLOG(LINFO) << "Using ICP with colour for pairwise registration";
		// Use ICP with colour to get "better" transformation.
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (prop_ICP_MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		// Set correspondence two-step correspondence estimation using colour.
		pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>);
		icp.setCorrespondenceEstimation(ceptr);

		// Add source and target clours.
		icp.setInputSource(src_cloud_xyzrgb_);
		icp.setInputTarget(trg_cloud_xyzrgb_);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_trg_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());

		// Align clouds.
		icp.align(*aligned_trg_cloud_rgbxyz);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		// Get the transformation from target to source.
		Types::HomogMatrix icp_trans = icp.getFinalTransformation();
		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		// Set resulting transformation.
		result.matrix() = icp_trans.matrix().inverse();

	} else if (!prop_ICP_SIFT) {
		CLOG(LINFO) << "Using stantard ICP for pairwise registration";
		// Use ICP to get "better" transformation.
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (prop_ICP_MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		// Add source and target clours.
		icp.setInputSource(src_cloud_xyzrgb_);
		icp.setInputTarget(trg_cloud_xyzrgb_);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_trg_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());

		// Align clouds.
		icp.align(*aligned_trg_cloud_rgbxyz);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

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
	CLOG(LTRACE)<< "pairwise_registration_xyzrgb";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;
	// Read current cloud from port.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_cloud_xyzrgb = in_trg_cloud_xyzrgb.read();

	// Check whether source cloud is received and is required to save.
	if ((!in_src_cloud_xyzrgb.empty()) && (!in_save_src_cloud_trigger.empty())) {
		CLOG(LINFO) << "Storing source cloud";
		src_cloud_xyzrgb = in_src_cloud_xyzrgb.read();
		in_save_src_cloud_trigger.read();
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
	}		//: else - !src_cloud_xyzrgb->empty ()

			// Return the transformation.
	if(!prop_ICP_SIFT) {
		out_transformation_xyzrgb.write(result);
	}
	else {
		out_transformation_xyzrgb.write(this->result);
	}
}

void PairwiseRegistration::pairwise_registration_xyzsift() {
	CLOG(LTRACE)<< "pairwise_registration_xyzsift";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;
	// Read current cloud from port.
	pcl::PointCloud<PointXYZSIFT>::Ptr trg_cloud_xyzsift = in_trg_cloud_xyzsift.read();
	CLOG(LINFO) << "Reading correspondences!";
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT>::Ptr correspondences = in_src_trg_correspondences.read();
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
		result = pairwise_icp_based_registration_xyzsift(src_cloud_xyzsift, trg_cloud_xyzsift, correspondences);
	}		//: else - !src_cloud_xyzrgb->empty ()

			// Return the transformation.
	CLOG(LINFO) << "transformation: " << result;
	this->result = result;
	out_transformation_xyzrgb.write(result);
}

} //: namespace PairwiseRegistration
} //: namespace Processors
