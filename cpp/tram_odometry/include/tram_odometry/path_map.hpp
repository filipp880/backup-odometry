// Polyline route map ("pathgraph"): arc-length parameterisation, heading,
// curvature and grade, plus a fast nearest-point projection.
//
// The map is a plain CSV/TSV file (x y z [s]) in the frame declared by
// path_map.frame_offset_{e,n} (params.hpp): offset 0 means UTM zone 37N metres,
// which is what artifacts/route/route_map.csv holds. Keeping the format trivial
// means we are not blocked while the official map format is being specified.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace tram {

struct PathPoint {
  double x = 0.0;         ///< easting, m, in the map frame
  double y = 0.0;         ///< northing, m, in the map frame
  double z = 0.0;         ///< altitude, m, absolute ellipsoidal
  double s = 0.0;         ///< arc length from the start of the map, m
  double heading = 0.0;   ///< rad, direction of travel, wrapped to (-pi, pi]
  double curvature = 0.0; ///< 1/m, positive = left turn
  double grade = 0.0;     ///< rad, path inclination, positive = climbing
  double cross_m = 0.0;   ///< signed cross-track, m, positive = left of travel
};

class PathMap {
 public:
  bool empty() const { return pts_.empty(); }
  size_t size() const { return pts_.size(); }
  const std::vector<PathPoint>& points() const { return pts_; }
  std::vector<PathPoint>& points() { return pts_; }

  /// Loads "x y z" or "x y z s" per line; '#' and '%' start a comment.
  /// Extra trailing columns are ignored, so the judge-frame files that carry
  /// tangents and curvatures load fine.
  bool loadCsv(const std::string& path);

  /// Recomputes s/heading/curvature/grade from the geometry (call after load).
  void finalise();

  /// Nearest-point projection of a query point.
  ///
  /// `out` receives the *map* point (its x/y lie on the polyline) together with
  /// the interpolated s/heading/curvature/grade and the signed cross-track in
  /// `out.cross_m`; positive means the query is to the left of the direction of
  /// travel, per REP-103. `along` and `cross` receive the same arc length and the
  /// unsigned cross-track distance, for callers that do not care about sign.
  ///
  /// Returns false if the map is empty, or if the projection is further away
  /// than max_radius when max_radius > 0.
  bool project(double x, double y, PathPoint& out, double& along, double& cross,
               double max_radius = 0.0) const;

  /// Interpolated point at arc length s. Clamps to the map ends, so a caller
  /// that must not silently freeze has to check s against totalLength() itself.
  bool pointAt(double s, PathPoint& out) const;

  double totalLength() const { return pts_.empty() ? 0.0 : pts_.back().s; }

  void clear();

 private:
  void buildIndex();
  int cellOf(double x, double y) const;

  std::vector<PathPoint> pts_;
  // Uniform grid (CSR-style) over the map bounding box for nearest search.
  double cell_ = 25.0;
  double min_x_ = 0.0, min_y_ = 0.0;
  int nx_ = 0, ny_ = 0;
  std::vector<int> cell_start_;
  std::vector<int> cell_items_;
};

}  // namespace tram
