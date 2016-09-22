/*
 * CorrespondanceEstimationSIFT.hpp
 *
 *  Created on: 10 lut 2016
 *      Author: mlepicka
 */

#ifndef CORRESPONDENCE_ESTIMATION_SIFT_H_
#define CORRESPONDENCE_ESTIMATION_SIFT_H_

#include <string>
#include <iostream>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>

namespace pcl {
namespace registration {
template<typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceEstimationSIFT: public CorrespondenceEstimation<PointSource,
		PointTarget, Scalar> {
public:
	typedef boost::shared_ptr<
			CorrespondenceEstimation<PointSource, PointTarget, Scalar> > Ptr;
	typedef boost::shared_ptr<
			const CorrespondenceEstimation<PointSource, PointTarget, Scalar> > ConstPtr;

	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
	using PCLBase<PointSource>::deinitCompute;

	typedef pcl::search::KdTree<PointTarget> KdTree;
	typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

	typedef pcl::PointCloud<PointSource> PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

	typedef pcl::PointCloud<PointTarget> PointCloudTarget;
	typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
	typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

	typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

	/** \brief Empty constructor. */
	CorrespondenceEstimationSIFT() {
		corr_name_ = "CorrespondenceEstimationSIFT";
	}

	/** \brief Empty destructor */
	virtual ~CorrespondenceEstimationSIFT() {
	}

	/** \brief Determine the correspondences between input and target cloud.
	 * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
	 * \param[in] max_distance maximum allowed distance between correspondences
	 */
	virtual void determineCorrespondences(pcl::Correspondences &correspondences,
			double max_distance = std::numeric_limits<double>::max()) {
		if (!initCompute())
			return;
		double max_xyz_dist_sqr = max_distance * max_distance;
		correspondences.resize(indices_->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		pcl::Correspondence corr;
		unsigned int nr_valid_correspondences = 0;
		int descriptor_size;
		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {

				std::vector<int> neigh_xyz_indices(1);
				std::vector<float> neigh_xyz_sqr_dists(1);

				if (!pcl_isfinite (input_->points[*idx].descriptor[0])) //skipping NaNs
				{
					continue;
				}

				const float * descriptor = input_->points[*idx].descriptor;

				descriptor_size = 128;

				int max_neighs = 5;

				int found_xyz_neighs = tree_->nearestKSearch(
						input_->points[*idx], max_neighs, neigh_xyz_indices,
						neigh_xyz_sqr_dists);

				float min_distance = neigh_xyz_sqr_dists[max_neighs - 1];
				float min_descriptor_distance = 999999;
				int min_idx = max_neighs - 1;

				for (int i = 0; i < neigh_xyz_indices.size(); i++) {

					const float* descriptor_t =
							target_->points[neigh_xyz_indices[i]].descriptor;

					float desc_distance = 0.0;

					for (int j = 0; j < descriptor_size; j++) {
						float diff = descriptor[j] - descriptor_t[j];
						desc_distance += diff * diff;
					}

					desc_distance = sqrt(desc_distance);

					if (desc_distance < min_descriptor_distance) {
						min_descriptor_distance = desc_distance;
						min_idx = i;
						min_distance = neigh_xyz_sqr_dists[i];
					}
				}

				if (min_distance > max_xyz_dist_sqr)
					continue;

				corr.index_query = *idx;
				corr.index_match = neigh_xyz_indices[min_idx];
				corr.distance = min_distance;
				correspondences[nr_valid_correspondences++] = corr;

			}
		} else {
			PointTarget pt;
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				// Copy the source data to a target PointTarget format so we can search in the tree
				pt = input_->points[*idx];

				tree_->nearestKSearch(pt, 1, index, distance);

				if (distance[0] > max_xyz_dist_sqr)
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
		}
		correspondences.resize(nr_valid_correspondences);

		deinitCompute();
	}

	/** \brief Determine the reciprocal correspondences between input and target cloud.
	 * A correspondence is considered reciprocal if both Src_i has Tgt_i as a
	 * correspondence, and Tgt_i has Src_i as one.
	 *
	 * \param[out] correspondences the found correspondences (index of query and target point, distance)
	 * \param[in] max_distance maximum allowed distance between correspondences
	 */
	virtual void determineReciprocalCorrespondences(
			pcl::Correspondences &correspondences, double max_distance =
					std::numeric_limits<double>::max()) {

		if (!initCompute())
			return;

		// setup tree for reciprocal search
		// Set the internal point representation of choice
		if (!initComputeReciprocal())
			return;
		double max_dist_sqr = max_distance * max_distance;

		correspondences.resize(indices_->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		std::vector<int> index_reciprocal(1);
		std::vector<float> distance_reciprocal(1);
		pcl::Correspondence corr;
		unsigned int nr_valid_correspondences = 0;
		int target_idx = 0;
		int descriptor_size;

		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {

				std::vector<int> neigh_xyz_indices(1);
				std::vector<float> neigh_xyz_sqr_dists(1);

				if (!pcl_isfinite (input_->points[*idx].descriptor[0])) //skipping NaNs
				{
					continue;
				}

				const float * descriptor = input_->points[*idx].descriptor;

				descriptor_size = 128;

				int max_neighs = 5;

				int found_xyz_neighs = tree_->nearestKSearch(
						input_->points[*idx], max_neighs, neigh_xyz_indices,
						neigh_xyz_sqr_dists);

				float min_distance = neigh_xyz_sqr_dists[max_neighs - 1];
				float min_descriptor_distance = 999999;
				int min_idx = max_neighs - 1;
				for (int i = 0; i < neigh_xyz_indices.size(); i++) {

					const float* descriptor_t =
							target_->points[neigh_xyz_indices[i]].descriptor;

					float desc_distance = 0.0;

					for (int j = 0; j < descriptor_size; j++) {
						float diff = descriptor[j] - descriptor_t[j];
						desc_distance += diff * diff;
					}

					desc_distance = sqrt(desc_distance);

					if (desc_distance < min_descriptor_distance) {
						min_descriptor_distance = desc_distance;
						min_idx = i;
						min_distance = neigh_xyz_sqr_dists[i];
					}
				}

				target_idx = min_idx;

				tree_reciprocal_->nearestKSearch(target_->points[target_idx],
						max_neighs, index_reciprocal, distance_reciprocal);

				float min_distance_reciprocal = distance_reciprocal[max_neighs
						- 1];
				float min_descriptor_distance_reciprocal = 999999;
				int min_idx_reciprocal = max_neighs - 1;
				const float * descriptor_reciprocal =
						target_->points[*idx].descriptor;

				for (int i = 0; i < index_reciprocal.size(); i++) {

					const float* descriptor_t =
							input_->points[index_reciprocal[i]].descriptor;

					float desc_distance = 0.0;

					for (int j = 0; j < descriptor_size; j++) {
						float diff = descriptor_reciprocal[j] - descriptor_t[j];
						desc_distance += diff * diff;
					}

					desc_distance = sqrt(desc_distance);

					if (desc_distance < min_descriptor_distance_reciprocal) {
						min_descriptor_distance_reciprocal = desc_distance;
						min_idx_reciprocal = i;
						min_distance_reciprocal = distance_reciprocal[i];
					}
				}

				if (min_idx != min_idx_reciprocal)
					continue;

				if (min_distance > max_distance)
					continue;

				corr.index_query = *idx;
				corr.index_match = neigh_xyz_indices[min_idx];
				corr.distance = min_distance;
				correspondences[nr_valid_correspondences++] = corr;

			}
		} else {
			PointTarget pt_src;
			PointSource pt_tgt;

			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				// Copy the source data to a target PointTarget format so we can search in the tree
				pt_src = input_->points[*idx];

				tree_->nearestKSearch(pt_src, 1, index, distance);
				//

				if (distance[0] > max_dist_sqr)
					continue;

				target_idx = index[0];

				// Copy the target data to a target PointSource format so we can search in the tree_reciprocal
				pt_tgt = target_->points[target_idx];

				tree_reciprocal_->nearestKSearch(pt_tgt, 1, index_reciprocal,
						distance_reciprocal);
				if (distance_reciprocal[0] > max_dist_sqr
						|| *idx != index_reciprocal[0])
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
		}
		correspondences.resize(nr_valid_correspondences);
		deinitCompute();
	}

	/** \brief Clone and cast to CorrespondenceEstimationBase */
	virtual boost::shared_ptr<
			CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > clone() const {
		Ptr copy(
				new CorrespondenceEstimation<PointSource, PointTarget, Scalar>(
						*this));
		return (copy);
	}
};
}
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /*CORRESPONDENCE_ESTIMATION_SIFT_H_ */

