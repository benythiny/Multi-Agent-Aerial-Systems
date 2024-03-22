#ifndef ASTAR_H_
#define ASTAR_H_

/**
 * @brief The 3D grid-based A* planner
 *
 * How to use:
 *
 * 1. // | --------------- Initialize the Astar object -------------- |
 *
 *    const double resolution = 0.6;
 *    astar::Astar astar(resolution);
 *
 * 2. // | ------------ Prepare a set for occupied cells ------------ |
 *
 *    std::set<astar::Cell> obstacles;
 *
 * ... fill in the obstacles, use the astar.toGrid() method to convert from real coordinates
 *     to the grid coordinates:
 *
 *    obstacles.insert(astar.toGrid(0.2, 10.3, -5.5));
 *    obstacles.insert(astar.toGrid(2, 3, 4));
 *
 * ... or insert the cells directly:
 *
 *    obstacles.insert(astar::Cell(5, 8, 2));
 *
 * 3. // | ----- Prepare start and goal (in real-world coords.) ----- |
 *
 *   astar::Position start(-3.2, -4.4, 3.4);
 *   astar::Position goal(10.2, 0.0, -5.5);
 *
 * 4. // | -------------------- Call the planner -------------------- |
 *
 * std::optional<std::list<astar::Position>> path = astar.plan(start, goal, obstacles);
 *
 * 5. // | ------------------- Process the result ------------------- |
 *
 *   if (path) {
 *     printf("path found:\n");
 *     for (astar::Position pos : path.value()) {
 *       printf("[%.2f, %.2f, %.2f]\n", pos.x(), pos.y(), pos.z());
 *     }
 *   } else {
 *     printf("path not found\n");
 *   }
 *
 */

/* includes //{ */

#include <ostream>
#include <math.h>
#include <iostream>
#include <cassert>
#include <optional>
#include <set>
#include <map>
#include <list>
#include <queue>

//}

namespace astar
{

// --------------------------------------------------------------
// |                       Support classes                      |
// --------------------------------------------------------------

/* class Position //{ */

/**
 * @brief Real-numbered 3D position with operations.
 */
class Position {

private:
  double x_ = 0;
  double y_ = 0;
  double z_ = 0;

public:
  Position(const double& x, const double& y, const double& z) : x_(x), y_(y), z_(z) {
  }

  // insertion operator, it enables the use of cout on this object: cout << Position(0, 0, 0) << endl;
  friend std::ostream& operator<<(std::ostream& os, const Position& pos) {

    os << "[" << pos.x_ << ", " << pos.y_ << ", " << pos.z_ << "]";
    return os;
  }

  double x() const {
    return x_;
  }

  double y() const {
    return y_;
  }

  double z() const {
    return z_;
  }

  bool operator==(const Position& rh) const {

    double x_diff = abs(this->x() - rh.x());
    double y_diff = abs(this->y() - rh.y());
    double z_diff = abs(this->z() - rh.z());

    return x_diff < 1e-3 && y_diff < 1e-3 && z_diff < 1e-3;
  }

  bool operator!=(const Position& rh) const {

    double x_diff = abs(this->x() - rh.x());
    double y_diff = abs(this->y() - rh.y());
    double z_diff = abs(this->z() - rh.z());

    return x_diff > 1e-3 || y_diff > 1e-3 || z_diff > 1e-3;
  }
};

//}

/* class Cell //{ */

const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                            {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                            {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                            {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};

/**
 * @brief Representation of a 3D grid cell for the A* planner.
 */
class Cell {

private:
  short coord_x_ = 0;
  short coord_y_ = 0;
  short coord_z_ = 0;

public:
  Cell() {
  }

  Cell(const int coord_x, const int coord_y, const int coord_z) {

    assert(coord_x > std::numeric_limits<short>::lowest() && coord_x < std::numeric_limits<short>::max());
    assert(coord_y > std::numeric_limits<short>::lowest() && coord_y < std::numeric_limits<short>::max());
    assert(coord_z > std::numeric_limits<short>::lowest() && coord_z < std::numeric_limits<short>::max());

    this->coord_x_ = static_cast<short>(coord_x);
    this->coord_y_ = static_cast<short>(coord_y);
    this->coord_z_ = static_cast<short>(coord_z);
  }

  Cell(const short coord_x, const short coord_y, const short coord_z) {
    this->coord_x_ = coord_x;
    this->coord_y_ = coord_y;
    this->coord_z_ = coord_z;
  }

  // insertion operator, it enables the use of cout on this object: cout << position(0,0) << endl;
  friend std::ostream& operator<<(std::ostream& os, const Cell& pos) {

    os << "[" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]";
    return os;
  }

  double distance(const Cell& rh) const {

    return sqrt(pow(coord_x_ - rh.coord_x_, 2) + pow(coord_y_ - rh.coord_y_, 2) + pow(coord_z_ - rh.coord_z_, 2));
    /* return abs(coord_x_ - rh.coord_x_) + abs(coord_y_ - rh.coord_y_) + abs(coord_z_ - rh.coord_z_); */
  }

  Cell operator+(const Cell& rh) const {

    const short x = static_cast<short>(this->coord_x_ + rh.coord_x_);
    const short y = static_cast<short>(this->coord_y_ + rh.coord_y_);
    const short z = static_cast<short>(this->coord_z_ + rh.coord_z_);

    return Cell(x, y, z);
  }

  // greater than operator
  bool operator>(const Cell& rh) const {

    unsigned long long this_x_offset = (unsigned long long)this->coord_x_ - std::numeric_limits<short>::lowest();
    unsigned long long this_y_offset = (unsigned long long)this->coord_y_ - std::numeric_limits<short>::lowest();
    unsigned long long this_z_offset = (unsigned long long)this->coord_z_ - std::numeric_limits<short>::lowest();

    unsigned long long rh_x_offset = (unsigned long long)rh.coord_x_ - std::numeric_limits<short>::lowest();
    unsigned long long rh_y_offset = (unsigned long long)rh.coord_y_ - std::numeric_limits<short>::lowest();
    unsigned long long rh_z_offset = (unsigned long long)rh.coord_z_ - std::numeric_limits<short>::lowest();

    return this_x_offset + (this_y_offset << 16) + (this_z_offset << 32) > rh_x_offset + (rh_y_offset << 16) + (rh_z_offset << 32);
  }

  // lesser than operator
  bool operator<(const Cell& rh) const {

    unsigned long long this_x_offset = (unsigned long long)this->coord_x_ - std::numeric_limits<short>::lowest();
    unsigned long long this_y_offset = (unsigned long long)this->coord_y_ - std::numeric_limits<short>::lowest();
    unsigned long long this_z_offset = (unsigned long long)this->coord_z_ - std::numeric_limits<short>::lowest();

    unsigned long long rh_x_offset = (unsigned long long)rh.coord_x_ - std::numeric_limits<short>::lowest();
    unsigned long long rh_y_offset = (unsigned long long)rh.coord_y_ - std::numeric_limits<short>::lowest();
    unsigned long long rh_z_offset = (unsigned long long)rh.coord_z_ - std::numeric_limits<short>::lowest();

    return this_x_offset + (this_y_offset << 16) + (this_z_offset << 32) < rh_x_offset + (rh_y_offset << 16) + (rh_z_offset << 32);
  }

  // equal comparison operator
  bool operator==(const Cell& rh) const {
    return ((x() == rh.x()) && (y() == rh.y()) && (z() == rh.z()));
  }

  // not equal comparison operator
  bool operator!=(const Cell& rh) const {
    return ((x() != rh.x()) || (y() != rh.y() || (z() != rh.z())));
  }

  Cell getNeighbour(const short x, const short y, const short z) const {
    Cell neighbour(this->x() + x, this->y() + y, this->z() + z);
    return neighbour;
  }

  std::list<Cell> expand(void) const {

    std::list<Cell> neighbours;

    for (auto& d : EXPANSION_DIRECTIONS) {
      Cell neighbour = getNeighbour(d[0], d[1], d[2]);
      neighbours.push_back(neighbour);
    }

    return neighbours;
  }

  short x(void) const {
    return coord_x_;
  }

  short y(void) const {
    return coord_y_;
  }

  short z(void) const {
    return coord_z_;
  }
};

//}

/* class DataCell //{ */

/**
 * @brief The Cell extended with a Value. Thanks to the value, the Cell can be added to ordered structures.
 */
template <typename T>
class DataCell : public Cell {

  using Cell::operator=;

private:
  T value_;

public:
  DataCell() : Cell() {
  }

  DataCell(const short coord_x, const short coord_y, const short coord_z, const T value) {
    this->coord_x_ = coord_x;
    this->coord_y_ = coord_y;
    this->coord_z_ = coord_z;
    this->value_   = value;
  }

  DataCell(Cell& rh, T value) : Cell(rh) {
    this->value_ = value;
  }

  DataCell(const Cell& rh, const T value) : Cell(rh) {
    this->value_ = value;
  }

  DataCell(const Cell& rh) : Cell(rh) {
  }

  DataCell(Cell& rh) : Cell(rh) {
  }

  T getValue(void) const {
    return (value_);
  }

  Cell getCell(void) const {
    return *this;
  }

  // greater than operator
  bool operator>(const DataCell& rh) const {

    return value_ > rh.getValue();
  }

  // lesser than operator
  bool operator<(const DataCell& rh) const {

    return value_ < rh.getValue();
  }

  // insertion operator, it enables the use of cout on this object: cout << position(0,0) << endl;
  friend std::ostream& operator<<(std::ostream& os, const DataCell& cell) {

    os << "[" << cell.x() << ", " << cell.y() << ", " << cell.z() << "](" << cell.getValue() << ")";
    return os;
  }

  // equal comparison operator
  bool operator==(const DataCell& rh) const {

    bool same_position = this->getCell() == rh.getCell();

    return same_position;
  }

  // not equal comparison operator
  bool operator!=(const DataCell& rh) const {

    bool different_position = this->getCell() != rh->getCell();

    return different_position;
  }

  bool operator==(const Cell& rh) const {
    return static_cast<Cell>(*this) == static_cast<Cell>(rh);
  }

  // not equal comparison operator
  bool operator!=(const Cell& rh) const {
    return static_cast<Cell>(*this) != static_cast<Cell>(rh);
  }
};

//}

/* class Grid //{ */

/**
 * @brief Grid operators: conversion form Cell to Position
 */
class Grid {

private:
  double resolution_ = 1.0;

  double center_x_ = 0;
  double center_y_ = 0;
  double center_z_ = 0;

public:
  Grid() {
  }

  Grid(const double resolution, const double center_x, const double center_y, const double center_z) {

    resolution_ = resolution;
    center_x_   = center_x;
    center_y_   = center_y;
    center_z_   = center_z;
  }

  Grid(const double &resolution) {

    assert(resolution > 0);

    resolution_ = resolution;
  }

  Cell toGrid(const Position& pos) const {

    int x_grid_int = (int)round(((double)pos.x() - center_x_) / resolution_);
    int y_grid_int = (int)round(((double)pos.y() - center_y_) / resolution_);
    int z_grid_int = (int)round(((double)pos.z() - center_z_) / resolution_);

    assert(x_grid_int > std::numeric_limits<short>::lowest() && x_grid_int < std::numeric_limits<short>::max());
    assert(y_grid_int > std::numeric_limits<short>::lowest() && y_grid_int < std::numeric_limits<short>::max());
    assert(z_grid_int > std::numeric_limits<short>::lowest() && z_grid_int < std::numeric_limits<short>::max());

    short x_grid = (short)x_grid_int;
    short y_grid = (short)y_grid_int;
    short z_grid = (short)z_grid_int;

    return Cell(x_grid, y_grid, z_grid);
  }

  Cell toGrid(const double& x, const double& y, const double& z) const {

    return toGrid(Position(x, y, z));
  }

  Position fromGrid(const Cell& cell) const {

    double x = ((double)cell.x()) * resolution_ + center_x_;
    double y = ((double)cell.y()) * resolution_ + center_y_;
    double z = ((double)cell.z()) * resolution_ + center_z_;

    return Position(x, y, z);
  }
};

//}

// --------------------------------------------------------------
// |                             A*                             |
// --------------------------------------------------------------

/* class AstarValue //{ */

/**
 * @brief Cell Value for the A* planner.
 */
class AstarValue {

private:
  double g_;
  double h_;

public:
  AstarValue() {
  }

  AstarValue(const double g, const double h) : g_(g), h_(h) {
  }

  bool operator<(const AstarValue& rh) const {

    double total_this = g_ + h_;
    double total_rh   = rh.g_ + rh.h_;

    if (total_this == total_rh) {
      return h_ < rh.h_;
    } else {
      return total_this < total_rh;
    }
  }

  bool operator>(const AstarValue& rh) const {

    double total_this = g_ + h_;
    double total_rh   = rh.g_ + rh.h_;

    if (total_this == total_rh) {
      return h_ > rh.h_;
    } else {
      return total_this > total_rh;
    }
  }

  bool operator==(const AstarValue& rh) const {
    return g_ + h_ == rh.g_ + rh.h_;
  }

  double g() const {
    return g_;
  }

  double h() const {
    return h_;
  }

  // insertion operator, it enables the use of cout on this object: cout << position(0,0) << endl;
  friend std::ostream& operator<<(std::ostream& os, const AstarValue& crit) {

    os << "g = " << crit.g_ << ", h = " << crit.h_ << ", f = " << crit.g_ + crit.h_;

    return os;
  }
};

//}

/* class Astar //{ */

/**
 * @brief The Astar class.
 */
class Astar {

private:
  Grid grid_;
  int  _max_iterations_;

public:
  /**
   * @brief convert from real coordinate to A* grid
   *
   * @param pos real 3D position
   *
   * @return A* grid cell
   */
  Cell toGrid(const Position& pos) const {
    return grid_.toGrid(pos);
  }

  /**
   * @brief convert from real coordinates to A* grid
   *
   * @param x x coordinate
   * @param y y coordinate
   * @param z z coordinate
   *
   * @return A* grid cell
   */
  Cell toGrid(const double& x, const double& y, const double& z) const {
    return grid_.toGrid(x, y, z);
  }

  /**
   * @brief convert from the A* grid to real coordinates
   *
   * @param cell
   *
   * @return grid cell
   */
  Position fromGrid(const Cell& cell) const {
    return grid_.fromGrid(cell);
  }

  /**
   * @brief initialize the A* solver object
   *
   * @param resolution the size of each grid cell
   * @param max_iterations maximum number of A* iterations before it cuts off
   */
  Astar(const double& resolution, const int& max_iterations = 1e7) {

    grid_            = Grid(resolution);
    _max_iterations_ = max_iterations;
  }

  /**
   * @brief plan the shortest path from start to goal.
   *
   * @param start_pos start Position (real-world coordinates)
   * @param goal_pos goal Position (real-world coordinates)
   * @param obstacles set of occupied cells
   *
   * @return the path from start to goal as an optional list of real-world Positions
   */
  std::optional<std::list<Position>> plan(const Position& start_pos, const Position& goal_pos, const std::set<Cell>& obstacles) const {

    Cell from_cell = toGrid(start_pos);
    Cell to_cell   = toGrid(goal_pos);

    /* std::cout << "[Astar]: planning from " << start_pos << ", which is " << from_cell << " in grid" << std::endl; */
    /* std::cout << "[Astar]: planning to " << goal_pos << ", which is " << to_cell << " in grid" << std::endl; */

    std::priority_queue<DataCell<AstarValue>, std::vector<DataCell<AstarValue>>, std::greater<DataCell<AstarValue>>> open_queue;

    std::map<Cell, Cell>   came_from;
    std::map<Cell, double> best_score;

    open_queue.push(DataCell(from_cell, AstarValue(0, from_cell.distance(to_cell))));

    came_from[from_cell]  = from_cell;
    best_score[from_cell] = 0;

    int                  n_expanded = 0;
    DataCell<AstarValue> current_cell;

    while (!open_queue.empty() && n_expanded < _max_iterations_) {

      n_expanded++;

      current_cell = open_queue.top();
      open_queue.pop();

      if (best_score.find(current_cell)->second < (current_cell.getValue().g())) {
        continue;
      }

      if (current_cell == to_cell) {
        break;
      }

      std::list<Cell> neighbours = current_cell.expand();

      for (Cell neighbour : neighbours) {

        if (obstacles.find(neighbour) != obstacles.end()) {
          continue;
        }

        double tentative_score = best_score.find(current_cell)->second + current_cell.distance(neighbour);

        auto neighbour_best_score = best_score.find(neighbour);

        double neighbour_score = std::numeric_limits<double>::max();

        if (neighbour_best_score != best_score.end()) {
          neighbour_score = neighbour_best_score->second;
        }

        if (tentative_score < neighbour_score) {

          AstarValue new_value(tentative_score, neighbour.distance(to_cell));

          open_queue.push(DataCell(neighbour, new_value));

          best_score[neighbour] = tentative_score;
          came_from[neighbour]  = current_cell;
        }
      }
    }

    /* std::cout << "[Astar]: Expanded " << n_expanded << " cells" << std::endl; */

    if (open_queue.empty() && current_cell != to_cell) {

      std::cout << "[Astar]: Open set is empty, but goal not found" << std::endl;
      return {};
    }

    if (!open_queue.empty() && current_cell != to_cell) {

      std::cout << "[Astar]: Open not empty, goal not found, " << n_expanded << " iterations passed" << std::endl;
      return {};
    }

    // backtrack

    Cell current = to_cell;

    std::list<Position> path;

    path.push_back(fromGrid(to_cell));

    while (current != from_cell) {
      current = came_from.find(current)->second;
      path.insert(path.begin(), fromGrid(current));
    }

    // insert the first and the last point, they might be different due to discretization

    if (start_pos != path.front()) {
      path.insert(path.begin(), start_pos);
    }

    if (goal_pos != path.back()) {
      path.push_back(goal_pos);
    }

    return {path};
  }
};

//}

}  // namespace astar

#endif  // ASTAR_H_
