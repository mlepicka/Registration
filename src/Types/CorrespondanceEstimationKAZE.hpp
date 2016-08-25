/*
 * CorrespondanceEstimationKAZE.hpp
 *
 *  Created on: 10 lut 2016
 *      Author: mlepicka
 */

#ifndef CORRESPONDENCE_ESTIMATION_KAZE_H_
#define CORRESPONDENCE_ESTIMATION_KAZE_H_

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
class CorrespondenceEstimationKAZE:
public CorrespondenceEstimation<PointSource, PointTarget, Scalar> {
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
	CorrespondenceEstimationKAZE() {
		corr_name_ = "CorrespondenceEstimationKAZE";
	}

	/** \brief Empty destructor */
	virtual ~CorrespondenceEstimationKAZE() {
	}

	/** \brief Determine the correspondences between input and target cloud.
	 * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
	 * \param[in] max_distance maximum allowed distance between correspondences
	 */
	virtual void determineCorrespondences(pcl::Correspondences &correspondences,
			double max_distance = std::numeric_limits<double>::max()) {

		if (!initCompute())
			return;
		double max_dist_sqr = max_distance * max_distance;
		correspondences.resize(indices_->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		pcl::Correspondence corr;
		unsigned int nr_valid_correspondences = 0;
		KAZEFeatureRepresentation::Ptr point_representation(
				new KAZEFeatureRepresentation());

		pcl::KdTreeFLANN<PointXYZKAZE> match_search;
		match_search.setPointRepresentation(point_representation);
		match_search.setInputCloud (target_);

		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {

				std::vector<int> neigh_indices(1);
				std::vector<float> neigh_sqr_dists(1);
				if (!pcl_isfinite (input_->points[*idx].descriptor[0])) //skipping NaNs
				{
					continue;
				}

				int found_neighs = match_search.nearestKSearch(input_->points[*idx],
						1, neigh_indices, neigh_sqr_dists);

				if (neigh_sqr_dists[0] > max_dist_sqr)
					continue;

//				if (found_neighs == 1) // && neigh_sqr_dists[0] < max_distance) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//				if(neigh_sqr_dists[0] < 0.25f)
//				{
				corr.index_query = *idx;
				corr.index_match = neigh_indices[0];
				corr.distance = neigh_sqr_dists[0];
				correspondences[nr_valid_correspondences++] = corr;
//				}

			}
		} else {
			PointTarget pt;
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				// Copy the source data to a target PointTarget format so we can search in the tree
				pt = input_->points[*idx];

				tree_->nearestKSearch(pt, 1, index, distance);

				if (distance[0] > max_dist_sqr)
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
		KAZEFeatureRepresentation::Ptr point_representation(
						new KAZEFeatureRepresentation());
		pcl::KdTreeFLANN<PointXYZKAZE> match_search;
		match_search.setPointRepresentation(point_representation);
		match_search.setInputCloud (target_);

		tree_reciprocal_->setPointRepresentation(point_representation);
		tree_->setPointRepresentation(point_representation);
		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {

				match_search.nearestKSearch(input_->points[*idx], 1, index, distance);

//				if (distance[0] > max_dist_sqr)
//					continue;

				target_idx = index[0];

				tree_reciprocal_->nearestKSearch(target_->points[target_idx], 1,
						index_reciprocal, distance_reciprocal);
				if ( *idx != index_reciprocal[0])
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
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

#endif /*CORRESPONDENCE_ESTIMATION_KAZE_H_ */

