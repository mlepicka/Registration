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



namespace Processors {
namespace PairwiseRegistration {

const double NORMALS_RADIUS = 0.04;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
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
	Base::Component(name),
	prop_ICP("Mode.ICP",true),
	prop_ICP_MaxCorrespondenceDistance("ICP.MaxCorrespondenceDistance",0.05),
	prop_ICP_MaximumIterations("ICP.MaximumIterations",50),
	prop_ICP_TransformationEpsilon("ICP.TransformationEpsilon",1e-8),
	prop_ICP_EuclideanFitnessEpsilon("ICP.EuclideanFitnessEpsilon",1),
	prop_ICP_colour("ICP.UseColour",true),
	prop_ICP_normals("ICP.UseNormals",true)
{
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
}

PairwiseRegistration::~PairwiseRegistration() {
}

void PairwiseRegistration::prepareInterface() {
	// Register data streams.
//	registerStream("in_cloud_xyz", &in_cloud_xyz); : TODO
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_previous_cloud_xyzrgb", &in_previous_cloud_xyzrgb);
	registerStream("out_transformation_xyz", &out_transformation_xyz);
	registerStream("out_transformation_xyzrgb", &out_transformation_xyzrgb);

	// Register handlers
	registerHandler("pairwise_registration_xyz", boost::bind(&PairwiseRegistration::pairwise_registration_xyz, this));
	addDependency("pairwise_registration_xyz", &in_cloud_xyz);

	registerHandler("pairwise_registration_xyzrgb", boost::bind(&PairwiseRegistration::pairwise_registration_xyzrgb, this));
	addDependency("pairwise_registration_xyzrgb", &in_cloud_xyzrgb);
}

bool PairwiseRegistration::onInit() {
	// Init prev cloud.
	previous_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

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

void  PairwiseRegistration::pairwise_registration_xyz(){
	CLOG(LTRACE) << "PairwiseRegistration::pariwise_registration_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	CLOG(LERROR) << "PairwiseRegistration::registration_xyz NOT IMPLEMENTED!";

	// Return identity matrix as transformation XYZ.
	Types::HomogMatrix result;
	result.setElements( Eigen::Matrix4f::Identity () );
	out_transformation_xyz.write(result);
}

void  PairwiseRegistration::pairwise_registration_xyzrgb(){
	CLOG(LTRACE) << "PairwiseRegistration::pariwise_registration_xyzrgb";
	// Temporary variable storing the resulting transformation.
	Types::HomogMatrix result;
	// Read current cloud from port.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();

	// Check whether previous cloud is received.
	if (!in_previous_cloud_xyzrgb.empty()) {
		CLOG(LINFO) << "Storing cloud as previous";
		previous_cloud_xyzrgb = in_previous_cloud_xyzrgb.read();
		// pcl::copyPointCloud<pcl::PointXYZRGB> (*cloud_xyzrgb, *previous_cloud_xyzrgb);
	}//: if

	// TODO: DEBUG - remove.
	CLOG(LERROR) << "Current cloud size:" << cloud_xyzrgb->size();
	if (!previous_cloud_xyzrgb->empty ())
			CLOG(LERROR) << "Previous cloud size:" << previous_cloud_xyzrgb->size();

	/// Previous cloud empty - initialization or ICP-based pairwise registration should not be used.
	if (previous_cloud_xyzrgb->empty () || !prop_ICP) {
		// Return identity matrix.
		CLOG(LINFO) << "ICP refinement not used";
		result.setElements( Eigen::Matrix4f::Identity () );
	} else {
		// Perform pairwise registration.
		if (prop_ICP_colour && prop_ICP_normals) {
			CLOG(LINFO) << "Using ICP with colour and normals for pairwise registration";

			// Compute surface normals.
			pcl::PointCloud<pcl::Normal>::Ptr tmp_cloud_normal (new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previous_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
			norm_est.setSearchMethod (tree);
			norm_est.setKSearch (30);
			//norm_est.setRadiusSearch(radius_search); // 0.05

			// Compute normals for previous cloud.
			norm_est.setInputCloud (previous_cloud_xyzrgb);
			norm_est.compute (*tmp_cloud_normal);
			// Concatenate clouds containing XYZRGB points and normals.
			pcl::concatenateFields(*previous_cloud_xyzrgb, *tmp_cloud_normal, *previous_cloud_xyzrgbnormal);

			// Compute normals for transformed cloud.
			norm_est.setInputCloud (cloud_xyzrgb);
			norm_est.compute (*tmp_cloud_normal);
			// Concatenate clouds containing XYZRGB points and normals.
			pcl::concatenateFields(*cloud_xyzrgb, *tmp_cloud_normal, *cloud_xyzrgbnormal);


			// Use ICP with normals AND colour to get "better" transformation.
//			pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
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

			icp.setInputSource (previous_cloud_xyzrgbnormal);
			icp.setInputTarget (cloud_xyzrgbnormal);

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		
			// Align clouds.
			icp.align(*aligned_cloud_xyzrgbnormal);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			result.setElements(icp_trans.inverse());
			out_transformation_xyzrgb.write(result);

		} else if (prop_ICP_normals) {
			CLOG(LINFO) << "Using ICP with normals for pairwise registration";


			// Compute surface normals.
			pcl::PointCloud<pcl::Normal>::Ptr tmp_cloud_normal (new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previous_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
			norm_est.setSearchMethod (tree);
			norm_est.setKSearch (30);
			//norm_est.setRadiusSearch(radius_search); // 0.05

			// Compute normals for previous cloud.
			norm_est.setInputCloud (previous_cloud_xyzrgb);
			norm_est.compute (*tmp_cloud_normal);
			// Concatenate clouds containing XYZRGB points and normals.
			pcl::concatenateFields(*previous_cloud_xyzrgb, *tmp_cloud_normal, *previous_cloud_xyzrgbnormal);

			// Compute normals for transformed cloud.
			norm_est.setInputCloud (cloud_xyzrgb);
			norm_est.compute (*tmp_cloud_normal);
			// Concatenate clouds containing XYZRGB points and normals.
			pcl::concatenateFields(*cloud_xyzrgb, *tmp_cloud_normal, *cloud_xyzrgbnormal);

			// Use ICP with normals to get "better" transformation.
//			pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
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

			icp.setInputSource (previous_cloud_xyzrgbnormal);
			icp.setInputTarget (cloud_xyzrgbnormal);

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_cloud_xyzrgbnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

/*
				    // Run the same optimization in a loop and visualize the results
				    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
				    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
				    reg_result = previous_cloud_xyzrgbnormals;

				    for (int i = 0; i < prop_ICP_MaximumIterations; ++i)
				    {
					CLOG(LINFO) << "ICP normals iteration:" << i;
				      // Estimate
					icp.setInputSource(reg_result);
					icp.align (*aligned_cloud_rgbxyzn);
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
					Eigen::Matrix4f icp_trans = Ti;*/

			// Align clouds.
			icp.align(*aligned_cloud_xyzrgbnormal);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation();

			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			result.setElements(icp_trans.inverse());
			out_transformation_xyzrgb.write(result);
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
			icp.setInputSource(previous_cloud_xyzrgb);
			icp.setInputTarget(cloud_xyzrgb);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());
		
			// Align clouds.
			icp.align(*aligned_cloud_rgbxyz);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			result.setElements(icp_trans.inverse());
			out_transformation_xyzrgb.write(result);

		} else {
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
			icp.setInputSource(previous_cloud_xyzrgb);
			icp.setInputTarget(cloud_xyzrgb);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>());

			// Align clouds.
			icp.align(*aligned_cloud_rgbxyz);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			result.setElements(icp_trans.inverse());

		}//: else ICP
	}//: else - !previous_cloud_xyzrgb->empty ()

	// Return the transformation.
	out_transformation_xyzrgb.write(result);
}



/*
http://www.pcl-users.org/Very-poor-registration-results-td3569265.html

http://pointclouds.org/documentation/tutorials/template_alignment.php#template-alignment

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr previous_cloud_xyznormals (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr transformed_cloud_xyznormals (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*previous_cloud_xyznormals);
  pcl::copyPointCloud (*src, *previous_cloud_xyznormals);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*transformed_cloud_xyznormals);
  pcl::copyPointCloud (*tgt, *transformed_cloud_xyznormals);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
*/

} //: namespace PairwiseRegistration
} //: namespace Processors
