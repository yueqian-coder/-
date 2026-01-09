#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void Astarpath::begin_grid_map(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GRID_X_SIZE = max_x_id;
  GRID_Y_SIZE = max_y_id;
  GRID_Z_SIZE = max_z_id;
  GLYZ_SIZE = GRID_Y_SIZE * GRID_Z_SIZE;
  GLXYZ_SIZE = GRID_X_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  data_raw = new uint8_t[GLXYZ_SIZE];
  memset(data_raw, 0, GLXYZ_SIZE * sizeof(uint8_t));

  Map_Node = new MappingNodePtr **[GRID_X_SIZE];
  for (int i = 0; i < GRID_X_SIZE; i++) {
    Map_Node[i] = new MappingNodePtr *[GRID_Y_SIZE];
    for (int j = 0; j < GRID_Y_SIZE; j++) {
      Map_Node[i][j] = new MappingNodePtr[GRID_Z_SIZE];
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        Map_Node[i][j][k] = new MappingNode(tmpIdx, pos);
      }
    }
  }
}

void Astarpath::setHeuristicWeight(double weight) {
  heuristic_weight_ = max(1.0, weight);
}

void Astarpath::setAraParams(bool enable, double init_weight, double step,
                             int max_iter) {
  use_ara_star_ = enable;
  ara_init_weight_ = max(1.0, init_weight);
  ara_step_ = max(0.0, step);
  ara_max_iter_ = max(1, max_iter);
}

void Astarpath::setThetaOptions(bool enable_theta, bool enable_lazy) {
  use_theta_star_ = enable_theta;
  use_lazy_theta_ = enable_lazy;
}

void Astarpath::setLineOfSightStep(double step) {
  los_step_ = step;
}

void Astarpath::setZPenalty(double penalty) {
  z_penalty_ = max(0.0, penalty);
}

void Astarpath::setThetaRewireZPenalty(double penalty) {
  theta_rewire_z_penalty_ = max(0.0, penalty);
}

void Astarpath::setUseRawOcc(bool enable) {
  use_raw_occ_ = enable;
}

void Astarpath::setOccInflateLevel(int level) {
  occ_inflate_level_ = std::min(2, std::max(0, level));
}

void Astarpath::resetGrid(MappingNodePtr ptr) {
  ptr->id = 0;
  ptr->Father = NULL;
  ptr->g_score = inf;
  ptr->f_score = inf;
}

void Astarpath::resetUsedGrids() {
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++)
        resetGrid(Map_Node[i][j][k]);
}

void Astarpath::set_barrier(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data_raw[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;

  data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;

  if (occ_inflate_level_ <= 0)
    return;

  auto mark = [&](int x, int y) {
    if (x < 0 || x >= GRID_X_SIZE || y < 0 || y >= GRID_Y_SIZE)
      return;
    data[x * GLYZ_SIZE + y * GRID_Z_SIZE + idx_z] = 1;
  };

  mark(idx_x + 1, idx_y);
  mark(idx_x - 1, idx_y);
  mark(idx_x, idx_y + 1);
  mark(idx_x, idx_y - 1);

  if (occ_inflate_level_ >= 2) {
    mark(idx_x + 1, idx_y + 1);
    mark(idx_x + 1, idx_y - 1);
    mark(idx_x - 1, idx_y + 1);
    mark(idx_x - 1, idx_y - 1);
  }
}

vector<Vector3d> Astarpath::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        // if(Map_Node[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (Map_Node[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(Map_Node[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d Astarpath::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Astarpath::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Vector3i Astarpath::c2i(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Eigen::Vector3d Astarpath::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool Astarpath::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

bool Astarpath::is_occupy(const Eigen::Vector3i &index) {
  return isOccupied(index(0), index(1), index(2));
}

bool Astarpath::is_occupy_raw(const Eigen::Vector3i &index) {
  int idx_x = index(0);
  int idx_y = index(1);
  int idx_z = index(2);
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data_raw[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] == 1));
}

inline bool Astarpath::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool Astarpath::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  if (idx_x < 0 || idx_x >= GRID_X_SIZE || idx_y < 0 || idx_y >= GRID_Y_SIZE ||
      idx_z < 0 || idx_z >= GRID_Z_SIZE)
    return false;
  const int idx = idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z;
  if (use_raw_occ_)
    return (data_raw[idx] == 1);
  return (data[idx] == 1);
}

inline bool Astarpath::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  if (idx_x < 0 || idx_x >= GRID_X_SIZE || idx_y < 0 || idx_y >= GRID_Y_SIZE ||
      idx_z < 0 || idx_z >= GRID_Z_SIZE)
    return false;
  const int idx = idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z;
  if (use_raw_occ_)
    return (data_raw[idx] < 1);
  return (data[idx] < 1);
}

inline void Astarpath::AstarGetSucc(MappingNodePtr currentPtr,
                                          vector<MappingNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i Idx_neighbor;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        Idx_neighbor(0) = (currentPtr->index)(0) + dx;
        Idx_neighbor(1) = (currentPtr->index)(1) + dy;
        Idx_neighbor(2) = (currentPtr->index)(2) + dz;

        if (Idx_neighbor(0) < 0 || Idx_neighbor(0) >= GRID_X_SIZE ||
            Idx_neighbor(1) < 0 || Idx_neighbor(1) >= GRID_Y_SIZE ||
            Idx_neighbor(2) < 0 || Idx_neighbor(2) >= GRID_Z_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            Map_Node[Idx_neighbor(0)][Idx_neighbor(1)][Idx_neighbor(2)]);
        double base_cost = sqrt(dx * dx + dy * dy + dz * dz);
        double z_penalty = z_penalty_ * abs(dz);
        edgeCostSets.push_back(base_cost + z_penalty);
      }
    }
  }
}

double Astarpath::getHeu(MappingNodePtr node1, MappingNodePtr node2) {
  
  // 娴ｈ法鏁ら弫鏉跨摟鐠烘繄顬囬崪灞肩缁夊秶琚崹瀣畱tie_breaker
  double heu;
  double tie_breaker;
  tie_breaker = 1.0 + 1.0 / 10000.0;
  heu = tie_breaker * heuristic_weight_ * (node1->coord - node2->coord).norm();
  return heu;
}

bool Astarpath::AstarSearch(Vector3d start_pt, Vector3d end_pt) {
  cache_path_valid_ = false;
  cached_path_.clear();
  double base_weight = heuristic_weight_;

  if (use_ara_star_) {
    bool found = false;
    double best_cost = inf;
    vector<Vector3d> best_path;
    double w = max(1.0, ara_init_weight_);
    for (int iter = 0; iter < ara_max_iter_ && w >= 1.0; ++iter) {
      heuristic_weight_ = w;
      if (AstarSearchSingle(start_pt, end_pt)) {
        vector<Vector3d> path = getPath();
        double cost = terminatePtr ? terminatePtr->g_score : inf;
        if (cost < best_cost) {
          best_cost = cost;
          best_path = path;
          found = true;
        }
      }
      resetUsedGrids();
      w = max(1.0, w - ara_step_);
    }
    heuristic_weight_ = base_weight;
    if (found) {
      cached_path_ = best_path;
      cache_path_valid_ = true;
      return true;
    }
    heuristic_weight_ = max(1.0, base_weight);
    if (AstarSearchSingle(start_pt, end_pt)) {
      cached_path_ = getPath();
      cache_path_valid_ = true;
      heuristic_weight_ = base_weight;
      return true;
    }
    heuristic_weight_ = base_weight;
    return false;
  }

  heuristic_weight_ = max(1.0, heuristic_weight_);
  bool ok = AstarSearchSingle(start_pt, end_pt);
  heuristic_weight_ = base_weight;
  return ok;
}


bool Astarpath::AstarSearchSingle(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  auto line_of_sight = [&](const Vector3d &a, const Vector3d &b) {
    Vector3d diff = b - a;
    double dist = diff.norm();
    if (dist < 1e-6)
      return true;
    double step = los_step_;
    if (step <= 0.0)
      step = max(0.05, resolution * 0.5);
    Vector3d dir = diff / dist;
    int steps = static_cast<int>(dist / step);
    Vector3d p = a;
    for (int i = 0; i <= steps; ++i) {
      if (isOccupied(coord2gridIndex(p)))
        return false;
      p += dir * step;
    }
    return true;
  };

  auto edge_cost = [&](const Vector3d &a, const Vector3d &b) {
    Vector3d diff = a - b;
    double base_cost = diff.norm();
    double z_penalty = theta_rewire_z_penalty_ * abs(diff(2));
    return base_cost + z_penalty;
  };

  // start_point 閸?end_point 缁便垹绱?
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  //start_point 閸?end_point 閻ㄥ嫪缍呯純?
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // 閸掓繂顫愰崠?struct MappingNode 閻ㄥ嫭瀵氶柦鍫礉閸掑棗鍩嗘禒锝堛€?start node 閸?goal node
  // 
  MappingNodePtr startPtr = new MappingNode(start_idx, start_pt);
  MappingNodePtr endPtr = new MappingNode(end_idx, end_pt);

  // Openset 閺勵垶鈧俺绻?STL 鎼存挷鑵戦惃?multimap 鐎圭偟骞囬惃鍒紁en_list
  Openset.clear();
  // currentPtr 鐞涖劎銇?open_list 娑?f閿涘潱閿?閺堚偓娴ｅ海娈戦懞鍌滃仯
  MappingNodePtr currentPtr = NULL;
  MappingNodePtr neighborPtr = NULL;

  // 鐏?Start 閼哄倻鍋ｉ弨鎯ф躬 Open Set 娑?
  startPtr->g_score = 0;
  /**
   *
   * STEP 1.1:  鐎瑰本鍨?Astarpath::getHeu
   *
   * **/
  startPtr->f_score = getHeu(startPtr, endPtr);

  

  startPtr->id = 1;
  startPtr->coord = start_pt;
  startPtr -> Father = NULL;
  Openset.insert(make_pair(startPtr->f_score, startPtr));


  double tentative_g_score;
  vector<MappingNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.2:  鐎瑰本鍨氬顏嗗箚
   *
   * **/

  while (!Openset.empty()) {
    //1.瀵懓鍤璯+h閺堚偓鐏忓繒娈戦懞鍌滃仯
        currentPtr = Openset.begin()->second;
    Openset.erase(Openset.begin());
    if (use_lazy_theta_ && currentPtr->Father &&
        !line_of_sight(currentPtr->Father->coord, currentPtr->coord)) {
      MappingNodePtr best_parent = NULL;
      double best_g = inf;
      vector<MappingNodePtr> lazy_neighbors;
      vector<double> lazy_costs;
      AstarGetSucc(currentPtr, lazy_neighbors, lazy_costs);
      for (unsigned int k = 0; k < lazy_neighbors.size(); k++) {
        MappingNodePtr nb = lazy_neighbors[k];
        if (nb->id != -1)
          continue;
        if (!line_of_sight(nb->coord, currentPtr->coord))
          continue;
        double g_through = nb->g_score + edge_cost(currentPtr->coord, nb->coord);
        if (g_through < best_g) {
          best_g = g_through;
          best_parent = nb;
        }
      }
      if (best_parent) {
        currentPtr->Father = best_parent;
        currentPtr->g_score = best_g;
      }
    }
    //2.閸掋倖鏌囬弰顖氭儊閺勵垳绮撻悙?
        if (currentPtr->index == goalIdx)
    {
      terminatePtr = currentPtr;
      return true;
    }
    //3.閹锋挸鐫嶈ぐ鎾冲閼哄倻鍋?
        currentPtr->id = -1;
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
    for(unsigned int i=0;i<neighborPtrSets.size();i++)
    {
      
      if(neighborPtrSets[i]->id==-1)
      {
         continue;
      }
            tentative_g_score = currentPtr->g_score + edgeCostSets[i];
      MappingNodePtr best_parent = currentPtr;
      neighborPtr = neighborPtrSets[i];
      if (use_theta_star_ && !use_lazy_theta_ && currentPtr->Father &&
          line_of_sight(currentPtr->Father->coord, neighborPtr->coord)) {
        double g_through_parent =
            currentPtr->Father->g_score +
            edge_cost(neighborPtr->coord, currentPtr->Father->coord);
        if (g_through_parent < tentative_g_score) {
          tentative_g_score = g_through_parent;
          best_parent = currentPtr->Father;
        }
      }
      if(isOccupied(neighborPtr->index))
      continue;
      if(neighborPtr->id==0)
      {
        //4.婵夘偄鍟撴穱鈩冧紖閿涘苯鐣幋鎰纯閺?
                neighborPtr->id = 1;
        neighborPtr->g_score = tentative_g_score;
        neighborPtr->f_score = tentative_g_score + getHeu(neighborPtr, endPtr);
        neighborPtr->Father = best_parent;
        neighborPtr->nodeMapIt =
          Openset.insert(make_pair(neighborPtr->f_score, neighborPtr));
        continue;
      }
      else if(neighborPtr->id==1)
      {
                if(neighborPtr->g_score > tentative_g_score){
          neighborPtr->g_score = tentative_g_score;
          neighborPtr->Father = best_parent;
          neighborPtr->f_score = tentative_g_score + getHeu(neighborPtr, endPtr);
          Openset.erase(neighborPtr->nodeMapIt);
          neighborPtr->nodeMapIt =
            Openset.insert(make_pair(neighborPtr->f_score, neighborPtr));
        }
      continue;
      }
    }
  }

  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}


vector<Vector3d> Astarpath::getPath() {
  if (cache_path_valid_)
    return cached_path_;
  vector<Vector3d> path;
  vector<MappingNodePtr> front_path;
  MappingNodePtr ptr = terminatePtr;
  while (ptr != NULL) {
    ptr->coord = gridIndex2coord(ptr->index);
    front_path.push_back(ptr);
    ptr = ptr->Father;
  }
  /**
   *
   * STEP 1.3:  鏉╄姤鍑介幍鎯у煂閻ㄥ嫯鐭惧?
   *
   * **/

    for (int i = static_cast<int>(front_path.size()) - 1; i >= 0; --i) {
    path.push_back(front_path[i]->coord);
  }

  return path;
}


std::vector<Vector3d> Astarpath::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {

  //init
  double dmax=0,d;
  int index=0;
  int end = path.size();
  //1.鐠侊紕鐣荤捄婵堫瀲妫ｆ牕鐔潻鐐村灇閻╁鍤庨張鈧径褏娈戦悙鐧哥礉楠炶泛鐨㈤悙褰掓肠娴犲孩顒濇径鍕瀻瀵偓
  for(int i=1;i<end-1;i++)
  {
    d=perpendicularDistance(path[i],path[0],path[end-1]);
    if(d>dmax)
    {
      index=i;
      dmax=d;
    }
  }
  vector<Vector3d> subPath1;
  int j = 0;
  while(j<index+1){
    subPath1.push_back(path[j]);
    j++;
  }
  vector<Vector3d> subPath2;
   while(j<int(path.size())){
    subPath2.push_back(path[j]);
    j++;
  }
  //2.閹峰棗鍨庨悙褰掓肠
  vector<Vector3d> recPath1;
  vector<Vector3d> recPath2;
  vector<Vector3d> resultPath;
  if(dmax>path_resolution)
  {
    recPath1=pathSimplify(subPath1,path_resolution);
    recPath2=pathSimplify(subPath2,path_resolution);
   for(int i=0;i<int(recPath1.size());i++){
    resultPath.push_back(recPath1[i]);
  }
     for(int i=0;i<int(recPath2.size());i++){
    resultPath.push_back(recPath2[i]);
  }
  }else{
    if(path.size()>1){
      resultPath.push_back(path[0]);
      resultPath.push_back(path[end-1]);
    }else{
      resultPath.push_back(path[0]);
    }
    
  }

  return resultPath;
}

double Astarpath::perpendicularDistance(const Eigen::Vector3d point_insert,const Eigen:: Vector3d point_st,const Eigen::Vector3d point_end)
{
  Vector3d line1=point_end-point_st;
  Vector3d line2=point_insert-point_st;
  return double(line2.cross(line1).norm()/line1.norm());
}

Vector3d Astarpath::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}


int Astarpath::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe

  double delta_t=resolution/1.0;//conservative advance step size;
  double t = delta_t;
  Vector3d advancePos;
  for(int i=0;i<polyCoeff.rows();i++)
  {
    while(t<time(i)){
     advancePos=getPosPoly(polyCoeff,i,t) ;
     if(isOccupied(coord2gridIndex(advancePos))){
       unsafe_segment=i;
       break;
     }   
     t+=delta_t;
    }
    if(unsafe_segment!=-1){

      break;
    }else{
      t=delta_t;
    }
  }
  return unsafe_segment;
}

void Astarpath::resetOccupy(){
  for (int i = 0; i < GRID_X_SIZE; i++)
for (int j = 0; j < GRID_Y_SIZE; j++)
  for (int k = 0; k < GRID_Z_SIZE; k++) {
    data[i * GLYZ_SIZE + j * GRID_Z_SIZE + k] = 0;
    data_raw[i * GLYZ_SIZE + j * GRID_Z_SIZE + k] = 0;
  }
}
