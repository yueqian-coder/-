#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class Astarpath
{	
	private:

	protected:
		uint8_t * data;
		uint8_t * data_raw;
		MappingNodePtr *** Map_Node;
		Eigen::Vector3i goalIdx;
		int GRID_X_SIZE, GRID_Y_SIZE, GRID_Z_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
		double heuristic_weight_;
		double ara_init_weight_;
		double ara_step_;
		double los_step_;
		double z_penalty_;
		double theta_rewire_z_penalty_;
		int ara_max_iter_;
		bool use_theta_star_;
		bool use_lazy_theta_;
		bool use_ara_star_;
		bool use_raw_occ_;
		int occ_inflate_level_;
		bool cache_path_valid_;
		std::vector<Eigen::Vector3d> cached_path_;

		MappingNodePtr terminatePtr;
		std::multimap<double, MappingNodePtr> Openset;

		double getHeu(MappingNodePtr node1, MappingNodePtr node2);
		bool AstarSearchSingle(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void AstarGetSucc(MappingNodePtr currentPtr, std::vector<MappingNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
		bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
    	
		
		

	public:
		Astarpath()
			: heuristic_weight_(1.0)
			, ara_init_weight_(1.6)
			, ara_step_(0.2)
			, los_step_(0.0)
			, z_penalty_(0.2)
			, theta_rewire_z_penalty_(0.0)
			, ara_max_iter_(3)
			, use_theta_star_(false)
			, use_lazy_theta_(false)
			, use_ara_star_(false)
			, use_raw_occ_(false)
			, occ_inflate_level_(2)
			, cache_path_valid_(false) {};
		~Astarpath(){};
		bool AstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void setHeuristicWeight(double weight);
		void setAraParams(bool enable, double init_weight, double step, int max_iter);
		void setThetaOptions(bool enable_theta, bool enable_lazy);
		void setLineOfSightStep(double step);
		void setZPenalty(double penalty);
		void setThetaRewireZPenalty(double penalty);
		void setUseRawOcc(bool enable);
		void setOccInflateLevel(int level);
		void resetGrid(MappingNodePtr ptr);
		void resetUsedGrids();
		bool is_occupy(const Eigen::Vector3i & index);
		bool is_occupy_raw(const Eigen::Vector3i & index);
		Eigen::Vector3i c2i(const Eigen::Vector3d & pt);

		void begin_grid_map(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void set_barrier(const double coord_x, const double coord_y, const double coord_z);

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
		std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path, const double path_resolution);
		Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
		int safeCheck( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
		double perpendicularDistance(const Eigen::Vector3d point_insert,const Eigen::Vector3d point_st,const Eigen::Vector3d point_end);
        void resetOccupy();
};


#endif
