#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>
#include <QPointF>
#include "graph.h"

static inline double haversine_m(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI/180.0;
    double dlon = (lon2 - lon1) * M_PI/180.0;
    double a = std::sin(dlat/2)*std::sin(dlat/2) +
               std::cos(lat1*M_PI/180.0)*std::cos(lat2*M_PI/180.0) *
               std::sin(dlon/2)*std::sin(dlon/2);
    return R * 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
}

struct ViewTransform {
    double minx=1e300, miny=1e300, maxx=-1e300, maxy=-1e300;
    double margin = 20.0, scale = 1.0, tx=0.0, ty=0.0;

    void compute(const Graph& g) {
        for (auto &n : g.nodes) {
            minx = std::min(minx, n.x); miny = std::min(miny, n.y);
            maxx = std::max(maxx, n.x); maxy = std::max(maxy, n.y);
        }
        double w = std::max(1.0, maxx - minx);
        double h = std::max(1.0, maxy - miny);
        scale = std::min((1000.0 - 2 * margin) / w, (800.0 - 2 * margin) / h);
        tx = -minx * scale + margin;
        ty = -miny * scale + margin;
    }

    QPointF toScene(double x, double y) const {
        return QPointF(x * scale + tx, y * scale + ty);
    }
};

#endif